#include "uart4_telemetry.h"
#include "slave_feedback.h"
#include "rs485.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define TELEMETRY_CONFIG_FLAG 0x01U
#define TELEMETRY_TX_TIMEOUT_MS 20U

// TelemetryConfig_t：保存一帧使用的完整配置，避免五指使用不同阈值
// 参数：比值和合力门槛均以十分之一为单位保存
// 返回值：无
typedef struct {
    uint16_t enabled;
    uint16_t frequency;
    uint16_t ratio_tenths;
    uint16_t force_min_tenths;
} TelemetryConfig_t;

volatile Uart4TelemetryStats_t uart4TelemetryStats;
static osThreadId_t s_task;
static uint8_t s_tx_buffer[UART4_TELEMETRY_FRAME_SIZE];
static volatile uint8_t s_tx_busy;
static volatile uint8_t s_tx_error;
static uint32_t s_tx_start_ms;

// Uart4Telemetry_ReadConfig：在短临界区内读取四个配置寄存器
// 参数：无
// 返回值：本次配置快照
static TelemetryConfig_t Uart4Telemetry_ReadConfig(void)
{
    TelemetryConfig_t config;
    taskENTER_CRITICAL();
    config.enabled = Reg[UART4_REPORT_REG_ENABLE];
    config.frequency = Reg[UART4_REPORT_REG_FREQUENCY];
    config.ratio_tenths = Reg[UART4_REPORT_REG_RATIO];
    config.force_min_tenths = Reg[UART4_REPORT_REG_FORCE_MIN];
    taskEXIT_CRITICAL();
    return config;
}

// Uart4Telemetry_PutU16：按大端编码一个16位字段
// 参数：out - 输出两字节位置；value - 待发送数值
// 返回值：无
static void Uart4Telemetry_PutU16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)value;
}

// Uart4Telemetry_Encode：生成帧头、六路角度、五路合力、状态和帧尾
// 参数：frame - 25字节输出区；snapshot - 反馈快照；config - 本帧配置
// 返回值：无；无效角度和力编码为0xFFFF，对应力状态清零
static void Uart4Telemetry_Encode(uint8_t frame[UART4_TELEMETRY_FRAME_SIZE],
                                 const SlaveFeedbackSnapshot_t *snapshot,
                                 const TelemetryConfig_t *config)
{
    memset(frame, 0, UART4_TELEMETRY_FRAME_SIZE);
    frame[0] = 0xFFU;
    for (uint8_t i = 0U; i < SLAVE_FEEDBACK_ANGLE_COUNT; ++i) {
        uint16_t angle = (snapshot->angle_valid_mask & (1U << i)) != 0U ?
                         snapshot->angle_tenths[i] : UART4_TELEMETRY_INVALID_VALUE;
        Uart4Telemetry_PutU16(&frame[1U + 2U * i], angle);
    }
    uint32_t minimum = config->force_min_tenths;
    uint32_t minimum_squared = minimum * minimum;
    for (uint8_t i = 0U; i < SLAVE_FEEDBACK_FORCE_COUNT; ++i) {
        uint16_t force = UART4_TELEMETRY_INVALID_VALUE;
        if ((snapshot->force_valid_mask & (1U << i)) != 0U) {
            uint32_t tangential = snapshot->tangential_tenths[i];
            uint32_t normal = snapshot->normal_tenths[i];
            uint32_t squared = tangential * tangential + normal * normal;
            // 分量本身已经是0.1N，平方根可直接得到同单位的合力原始值。
            force = (uint16_t)(sqrtf((float)squared) + 0.5f);
            if (tangential > 0U && squared >= minimum_squared &&
                10U * normal < (uint32_t)config->ratio_tenths * tangential) {
                frame[23] |= (uint8_t)(1U << i);
            }
        }
        Uart4Telemetry_PutU16(&frame[13U + 2U * i], force);
    }
    frame[24] = 0xAAU;
}

// Uart4Telemetry_Send：只在配置仍匹配且发送空闲时提交完整帧
// 参数：config - 编码依据的配置快照
// 返回值：无；配置已变化时丢弃本次编码，等待任务处理通知
static void Uart4Telemetry_Send(const TelemetryConfig_t *config)
{
    SlaveFeedbackSnapshot_t snapshot;
    uint8_t frame[UART4_TELEMETRY_FRAME_SIZE];
    SlaveFeedback_GetSnapshot(&snapshot);
    Uart4Telemetry_Encode(frame, &snapshot, config);
    taskENTER_CRITICAL();
    // 与MODBUS配置提交互斥，确保关闭应答之后不会再启动旧配置帧。
    if (Reg[UART4_REPORT_REG_ENABLE] == 1U &&
        Reg[UART4_REPORT_REG_FREQUENCY] == config->frequency &&
        Reg[UART4_REPORT_REG_RATIO] == config->ratio_tenths &&
        Reg[UART4_REPORT_REG_FORCE_MIN] == config->force_min_tenths) {
        if (s_tx_busy != 0U || huart4.gState != HAL_UART_STATE_READY) {
            uart4TelemetryStats.busy_skips++;
        } else {
            memcpy(s_tx_buffer, frame, sizeof(s_tx_buffer));
            s_tx_start_ms = HAL_GetTick();
            s_tx_busy = 1U;
            if (HAL_UART_Transmit_IT(&huart4, s_tx_buffer, sizeof(s_tx_buffer)) != HAL_OK) {
                s_tx_busy = 0U;
                uart4TelemetryStats.tx_errors++;
            }
        }
    }
    taskEXIT_CRITICAL();
}

// Uart4Telemetry_NextInterval：通过余数累计保持整数Hz的长期平均周期
// 参数：frequency - 1～100Hz；fraction - 周期分配余数
// 返回值：下一周期的内核节拍数，至少为1
static uint32_t Uart4Telemetry_NextInterval(uint16_t frequency, uint32_t *fraction)
{
    uint32_t tick_rate = osKernelGetTickFreq();
    uint32_t interval = tick_rate / frequency;
    *fraction += tick_rate % frequency;
    if (*fraction >= frequency) {
        *fraction -= frequency;
        interval++;
    }
    return interval > 0U ? interval : 1U;
}

// Uart4Telemetry_Task：定时从缓存编码并非阻塞发送，配置通知可提前唤醒
// 参数：argument - 未使用
// 返回值：无，任务持续运行
static void Uart4Telemetry_Task(void *argument)
{
    (void)argument;
    uint32_t fraction = 0U;
    uint32_t deadline = osKernelGetTickCount();
    uint32_t maintenance = osKernelGetTickFreq() / 100U;
    if (maintenance == 0U) maintenance = 1U;
    TelemetryConfig_t applied = {0};
    for (;;) {
        TelemetryConfig_t config = Uart4Telemetry_ReadConfig();
        uint32_t now = osKernelGetTickCount();
        if (s_tx_error != 0U || (s_tx_busy != 0U &&
            (uint32_t)(HAL_GetTick() - s_tx_start_ms) >= TELEMETRY_TX_TIMEOUT_MS)) {
            // 在任务内终止失常发送，HAL会关闭发送中断并恢复空闲状态。
            (void)HAL_UART_AbortTransmit(&huart4);
            s_tx_error = 0U;
            s_tx_busy = 0U;
            uart4TelemetryStats.tx_errors++;
        }
        if (config.enabled != applied.enabled || config.frequency != applied.frequency) {
            fraction = 0U;
            deadline = now + Uart4Telemetry_NextInterval(config.frequency, &fraction);
        }
        applied = config;
        if (config.enabled == 1U && (int32_t)(now - deadline) >= 0) {
            Uart4Telemetry_Send(&config);
            deadline += Uart4Telemetry_NextInterval(config.frequency, &fraction);
            // 任务曾被长时间抢占时舍弃错过周期，防止恢复后突发补发旧帧。
            now = osKernelGetTickCount();
            if ((int32_t)(now - deadline) >= 0) {
                deadline = now + Uart4Telemetry_NextInterval(config.frequency, &fraction);
            }
        }
        uint32_t wait_ticks = maintenance;
        if (config.enabled == 1U) {
            uint32_t remaining = deadline - osKernelGetTickCount();
            if ((int32_t)remaining <= 0) wait_ticks = 1U;
            else if (remaining < wait_ticks) wait_ticks = remaining;
        }
        (void)osThreadFlagsWait(TELEMETRY_CONFIG_FLAG, osFlagsWaitAny, wait_ticks);
    }
}

// Uart4Telemetry_Init：创建低于推杆控制优先级的上报任务
// 参数：无
// 返回值：true表示创建成功
bool Uart4Telemetry_Init(void)
{
    const osThreadAttr_t attr = {
        .name = "Uart4Telemetry", .stack_size = 1536U,
        .priority = osPriorityBelowNormal
    };
    s_task = osThreadNew(Uart4Telemetry_Task, NULL, &attr);
    return s_task != NULL;
}

// Uart4Telemetry_NotifyConfigChanged：在寄存器提交后发送任务标志
// 参数：无
// 返回值：无；任务尚未建立时会在启动时读取默认配置
void Uart4Telemetry_NotifyConfigChanged(void)
{
    if (s_task != NULL) (void)osThreadFlagsSet(s_task, TELEMETRY_CONFIG_FLAG);
}

// Uart4Telemetry_OnTxComplete：在最后一个停止位发送完后释放缓冲区
// 参数：无
// 返回值：无
void Uart4Telemetry_OnTxComplete(void)
{
    s_tx_busy = 0U;
    uart4TelemetryStats.sent_frames++;
}

// Uart4Telemetry_OnTxError：只置错误标志，不在中断内执行恢复
// 参数：无
// 返回值：无
void Uart4Telemetry_OnTxError(void)
{
    s_tx_error = 1U;
}
