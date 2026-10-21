#include "can_telemetry.h"
#include "can.h"
#include "rs485.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define CAN_CONFIG_CHANGED_FLAG 0x01U
#define CAN_MAINTENANCE_TICKS 5U
#define CAN_MAILBOX_TIMEOUT_MS 5U
#define CAN_INIT_RETRY_MS 1000U

volatile CanTelemetryStats_t canTelemetryStats;
static osThreadId_t s_task;
static uint32_t s_pending_mask;
static uint32_t s_pending_tick[3];
static uint32_t s_last_init_tick;
static uint8_t s_started;
static uint8_t s_bus_off_seen;
static uint8_t s_sequence;
static uint16_t s_applied_frequency = CAN_REPORT_DEFAULT_FREQUENCY;

// CanTelemetry_ForceByte：将非负力量化到0.1牛顿并饱和到255
// 参数：force - 牛顿值；非有限值由调用者通过有效位拒绝
// 返回值：0～255，其中255仍是合法的25.5牛顿
static uint8_t CanTelemetry_ForceByte(float force)
{
    if (!isfinite(force) || force <= 0.0f) return 0U;
    if (force >= 25.5f) return 255U;
    return (uint8_t)(force * 10.0f + 0.5f);
}

// CanTelemetry_Encode：使用缓存中的成品角度与力，不重新采样或计算三轴合成
// 参数：snapshot - 同一采集快照；now_tick - HAL时刻；sequence - 共同序号；data为输出
// 返回值：true表示三指角度全部有效，false时只允许发送带有效位的力帧
bool CanTelemetry_Encode(const MotorTelemetrySnapshot_t *snapshot, uint32_t now_tick,
                         uint8_t sequence, uint8_t angle_data[7], uint8_t force_data[8])
{
    if (snapshot == NULL || angle_data == NULL || force_data == NULL) return false;
    memset(angle_data, 0, 7U);
    memset(force_data, 0, 8U);
    bool angle_valid = SensorFeedback_IsFresh(snapshot->adc_valid,
                                              snapshot->adc_update_tick, now_tick);
    for (uint8_t channel = 0U; channel < SENSOR_FORCE_COUNT; channel++) {
        uint8_t motor_index = tactile_motor_indices[channel];
        uint16_t angle = snapshot->joint_angle_tenths[motor_index];
        if ((snapshot->angle_valid_mask & (uint8_t)(1U << motor_index)) == 0U) {
            angle_valid = false;
        }
        if (angle > 900U) angle = 900U;
        angle_data[2U * channel] = (uint8_t)(angle >> 8);
        angle_data[2U * channel + 1U] = (uint8_t)angle;
        const SensorForceFeedback_t *force = &snapshot->force[channel];
        force_data[2U * channel] = CanTelemetry_ForceByte(force->tangential_force);
        force_data[2U * channel + 1U] = CanTelemetry_ForceByte(force->normal_force);
        if (SensorFeedback_IsFresh(force->valid, force->last_update_tick, now_tick) &&
            isfinite(force->tangential_force) && isfinite(force->normal_force)) {
            force_data[6] |= (uint8_t)(1U << channel);
        }
    }
    angle_data[6] = sequence;
    force_data[7] = sequence;
    return angle_valid;
}

// CanTelemetry_Init：禁用业务接收过滤器并启动CAN，保留外设现有位时序
// 参数：无
// 返回值：HAL启动状态；全部失败均留给本任务重试
HAL_StatusTypeDef CanTelemetry_Init(void)
{
    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_RESET ||
        HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_ERROR) {
        MX_CAN1_Init();
    }
    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING) {
        s_started = 1U;
        return HAL_OK;
    }
    if (HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_READY) return HAL_ERROR;
    CAN_FilterTypeDef filter = {0};
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = DISABLE;
    filter.SlaveStartFilterBank = 14U;
    // 当前只上报，不接收查询；不用全接收过滤器制造无人消费的RX FIFO。
    for (uint32_t bank = 0U; bank < 14U; bank++) {
        filter.FilterBank = bank;
        if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) return HAL_ERROR;
    }
    HAL_StatusTypeDef status = HAL_CAN_Start(&hcan1);
    s_started = status == HAL_OK ? 1U : 0U;
    return status;
}

// CanTelemetry_CancelPending：撤销已入邮箱但尚未完成的旧反馈
// 参数：无；只允许CAN任务调用，避免多个任务同时操作HAL发送状态
// 返回值：无；不等待撤销完成
static void CanTelemetry_CancelPending(void)
{
    if (s_started != 0U && s_pending_mask != 0U) {
        (void)HAL_CAN_AbortTxRequest(&hcan1, s_pending_mask);
        for (uint8_t i = 0U; i < 3U; i++) {
            if ((s_pending_mask & (1UL << i)) != 0U) canTelemetryStats.aborted_frames++;
        }
    }
    s_pending_mask = 0U;
}

// CanTelemetry_Maintain：处理邮箱完成、超时和总线关闭状态
// 参数：now_ms - 当前HAL时刻
// 返回值：无；自动Bus-Off恢复由CAN硬件完成
static void CanTelemetry_Maintain(uint32_t now_ms)
{
    if (s_started == 0U) {
        if ((uint32_t)(now_ms - s_last_init_tick) >= CAN_INIT_RETRY_MS) {
            s_last_init_tick = now_ms;
            if (CanTelemetry_Init() != HAL_OK) canTelemetryStats.init_failures++;
        }
        return;
    }
    uint32_t esr = READ_REG(hcan1.Instance->ESR);
    canTelemetryStats.last_esr = esr;
    if ((esr & CAN_ESR_BOFF) != 0U) {
        if (s_bus_off_seen == 0U) {
            canTelemetryStats.bus_off_count++;
            CanTelemetry_CancelPending();
        }
        s_bus_off_seen = 1U;
    } else {
        s_bus_off_seen = 0U;
    }
    for (uint8_t i = 0U; i < 3U; i++) {
        uint32_t mailbox = 1UL << i;
        if ((s_pending_mask & mailbox) == 0U) continue;
        if (HAL_CAN_IsTxMessagePending(&hcan1, mailbox) == 0U) {
            if ((READ_REG(hcan1.Instance->TSR) & (CAN_TSR_TXOK0 << (8U * i))) == 0U) {
                canTelemetryStats.tx_failures++;
            }
            s_pending_mask &= ~mailbox;
        } else if ((uint32_t)(now_ms - s_pending_tick[i]) >= CAN_MAILBOX_TIMEOUT_MS) {
            (void)HAL_CAN_AbortTxRequest(&hcan1, mailbox);
            canTelemetryStats.aborted_frames++;
            s_pending_mask &= ~mailbox;
        }
    }
}

// CanTelemetry_Submit：非阻塞加入一个标准数据帧并登记所属邮箱
// 参数：id - 标准ID；length - 字节数；data - 最多8字节；now_ms - 当前时刻
// 返回值：无；失败只累计错误，不重发旧样本
static void CanTelemetry_Submit(uint32_t id, uint32_t length, const uint8_t *data,
                                uint32_t now_ms)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox = 0U;
    uint8_t payload[8] = {0};
    memcpy(payload, data, length); // HAL内部读取两组32位数据，7字节帧也提供完整8字节空间
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    header.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, payload, &mailbox) != HAL_OK) {
        canTelemetryStats.tx_failures++;
        return;
    }
    s_pending_mask |= mailbox;
    for (uint8_t i = 0U; i < 3U; i++) {
        if ((mailbox & (1UL << i)) != 0U) s_pending_tick[i] = now_ms;
    }
}

// CanTelemetry_Process：同一上报周期只取一次快照，两帧共用序号
// 参数：now_tick - HAL毫秒时刻
// 返回值：无；不等待总线、不触发传感器读取
void CanTelemetry_Process(uint32_t now_tick)
{
    MotorTelemetrySnapshot_t snapshot;
    uint8_t angle_data[7], force_data[8];
    Motor_GetTelemetrySnapshot(&snapshot);
    bool angle_valid = CanTelemetry_Encode(&snapshot, now_tick, s_sequence++,
                                            angle_data, force_data);
    if (s_started == 0U || (READ_REG(hcan1.Instance->ESR) & CAN_ESR_BOFF) != 0U) {
        canTelemetryStats.skipped_groups++;
        return;
    }
    taskENTER_CRITICAL();
    // 与MODBUS的开关提交互斥，避免写入停止后又提交一组新帧。
    if (Reg[CAN_REPORT_REG_ENABLE] != 0U &&
        Reg[CAN_REPORT_REG_FREQUENCY] == s_applied_frequency &&
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= (angle_valid ? 2U : 1U)) {
        if (angle_valid) CanTelemetry_Submit(CAN_TELEMETRY_ANGLE_ID, 7U, angle_data, now_tick);
        CanTelemetry_Submit(CAN_TELEMETRY_FORCE_ID, 8U, force_data, now_tick);
    } else {
        canTelemetryStats.skipped_groups++;
    }
    taskEXIT_CRITICAL();
}

// CanTelemetry_NotifyConfigChanged：用线程标志唤醒任务，避免低频时迟迟不能停止
// 参数：无
// 返回值：无；任务尚未创建时，启动任务会直接读取寄存器
void CanTelemetry_NotifyConfigChanged(void)
{
    if (s_task != NULL) (void)osThreadFlagsSet(s_task, CAN_CONFIG_CHANGED_FLAG);
}

// CanTelemetry_NextInterval：用余数累计生成给定Hz的内核tick间隔
// 参数：frequency - 1～100 Hz；fraction - 上轮余数，配置改变时清零
// 返回值：本次至少1个tick，长期平均频率不因整数除法产生偏差
static uint32_t CanTelemetry_NextInterval(uint16_t frequency, uint32_t *fraction)
{
    uint32_t tick_rate = osKernelGetTickFreq();
    uint32_t interval = tick_rate / frequency;
    *fraction += tick_rate % frequency;
    if (*fraction >= frequency) {
        *fraction -= frequency;
        interval++;
    }
    return interval != 0U ? interval : 1U;
}

// vCanTelemetryTask：按公共频率上报，并每5个tick维护一次总线与邮箱
// 参数：argument - 未使用
// 返回值：无，持续运行；所有CAN失败均限制在本任务内
void vCanTelemetryTask(void *argument)
{
    (void)argument;
    s_task = osThreadGetId();
    uint16_t enabled, frequency;
    Modbus_GetCanReportConfig(&enabled, &frequency);
    s_applied_frequency = frequency;
    uint32_t fraction = 0U;
    uint32_t next_tick = osKernelGetTickCount() + CanTelemetry_NextInterval(frequency, &fraction);
    s_last_init_tick = HAL_GetTick();
    if (CanTelemetry_Init() != HAL_OK) canTelemetryStats.init_failures++;
    for (;;) {
        CanTelemetry_Maintain(HAL_GetTick());
        uint16_t new_enabled, new_frequency;
        Modbus_GetCanReportConfig(&new_enabled, &new_frequency);
        if (new_enabled != enabled || new_frequency != frequency) {
            CanTelemetry_CancelPending();
            enabled = new_enabled;
            frequency = new_frequency;
            s_applied_frequency = frequency;
            fraction = 0U;
            next_tick = osKernelGetTickCount() + CanTelemetry_NextInterval(frequency, &fraction);
        }
        uint32_t now_tick = osKernelGetTickCount();
        if (enabled != 0U && (int32_t)(now_tick - next_tick) >= 0) {
            CanTelemetry_Process(HAL_GetTick());
            next_tick += CanTelemetry_NextInterval(frequency, &fraction);
            if ((int32_t)(osKernelGetTickCount() - next_tick) >= 0) {
                // 调度超期只发送最新一组，跳过旧周期，禁止突发补发。
                next_tick = osKernelGetTickCount() + CanTelemetry_NextInterval(frequency, &fraction);
            }
        }
        uint32_t wait_ticks = CAN_MAINTENANCE_TICKS;
        now_tick = osKernelGetTickCount();
        if (enabled != 0U) {
            int32_t remaining = (int32_t)(next_tick - now_tick);
            if (remaining <= 0) wait_ticks = 1U;
            else if ((uint32_t)remaining < wait_ticks) wait_ticks = (uint32_t)remaining;
        }
        (void)osThreadFlagsWait(CAN_CONFIG_CHANGED_FLAG, osFlagsWaitAny, wait_ticks);
    }
}
