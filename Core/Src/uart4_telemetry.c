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
static osMutexId_t s_config_mutex;
// 发送缓冲放在DMA可访问的普通SRAM，不能移入CCMRAM；保持到UART最后一个停止位发送完成。
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

// Uart4Telemetry_Send：用任务互斥锁校验配置并提交整帧DMA发送，不屏蔽全局中断
// 参数：config - 编码依据的配置快照
// 返回值：无；配置已变化时丢弃本次编码，等待任务处理通知
static void Uart4Telemetry_Send(const TelemetryConfig_t *config)
{
    SlaveFeedbackSnapshot_t snapshot;
    uint8_t frame[UART4_TELEMETRY_FRAME_SIZE];
    SlaveFeedback_GetSnapshot(&snapshot);
    Uart4Telemetry_Encode(frame, &snapshot, config);
    if (!Uart4Telemetry_LockConfig()) return;
    // 与MODBUS配置提交串行化，锁只持有到DMA启动，不等待约1.085ms的线路发送。
    if (Reg[UART4_REPORT_REG_ENABLE] == 1U &&
        Reg[UART4_REPORT_REG_FREQUENCY] == config->frequency &&
        Reg[UART4_REPORT_REG_RATIO] == config->ratio_tenths &&
        Reg[UART4_REPORT_REG_FORCE_MIN] == config->force_min_tenths) {
        if (s_tx_busy != 0U || huart4.gState != HAL_UART_STATE_READY) {
            uart4TelemetryStats.busy_skips++;
        } else {
            // 上报任务是唯一发送者；确认空闲后才覆盖缓冲，复制过程允许其他中断运行。
            memcpy(s_tx_buffer, frame, sizeof(s_tx_buffer));
            s_tx_start_ms = HAL_GetTick();
            s_tx_busy = 1U;
            // 只暂缓本路DMA中断，避免HAL默认开启的半传输中断抢先进入；其他中断不受影响。
            HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);
            HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart4, s_tx_buffer, sizeof(s_tx_buffer));
            if (status == HAL_OK) {
                // 一帧只需要DMA传输完成和UART线路发送完成，不需要半帧通知。
                __HAL_DMA_DISABLE_IT(huart4.hdmatx, DMA_IT_HT);
            } else {
                s_tx_busy = 0U;
                uart4TelemetryStats.tx_errors++;
            }
            HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
        }
    }
    Uart4Telemetry_UnlockConfig();
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
            // 在任务内同步终止失常DMA和UART发送，硬件停止读取缓冲后才允许下一帧覆盖。
            HAL_StatusTypeDef abort_status = HAL_UART_AbortTransmit(&huart4);
            if ((huart4.hdmatx->Instance->CR & DMA_SxCR_EN) != 0U) {
                // HAL可能因DMA停止超时提前返回；下次维护周期继续尝试停止该通道。
                (void)HAL_DMA_Abort(huart4.hdmatx);
            }
            bool stopped = abort_status == HAL_OK &&
                (huart4.hdmatx->Instance->CR & DMA_SxCR_EN) == 0U;
            // 未确认DMA停下时继续占用缓冲，防止覆盖硬件仍可能读取的数据。
            s_tx_error = stopped ? 0U : 1U;
            s_tx_busy = stopped ? 0U : 1U;
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

// Uart4Telemetry_Init：创建配置互斥锁和低于推杆控制优先级的上报任务
// 参数：无
// 返回值：true表示创建成功
bool Uart4Telemetry_Init(void)
{
    const osMutexAttr_t mutex_attr = {
        .name = "Uart4Config", .attr_bits = osMutexPrioInherit
    };
    s_config_mutex = osMutexNew(&mutex_attr);
    if (s_config_mutex == NULL) return false;
    const osThreadAttr_t attr = {
        .name = "Uart4Telemetry", .stack_size = 1536U,
        .priority = osPriorityBelowNormal
    };
    s_task = osThreadNew(Uart4Telemetry_Task, NULL, &attr);
    return s_task != NULL;
}

// Uart4Telemetry_LockConfig：串行化上报配置提交与DMA启动，等待期间允许中断和其他任务运行
// 参数：无；只能由任务调用，且不能在临界区内调用
// 返回值：true表示取得互斥锁，false表示未初始化或锁获取失败
bool Uart4Telemetry_LockConfig(void)
{
    return s_config_mutex != NULL && osMutexAcquire(s_config_mutex, osWaitForever) == osOK;
}

// Uart4Telemetry_UnlockConfig：释放配置与DMA启动共用的任务互斥锁
// 参数：无；仅由已成功取得互斥锁的任务调用
// 返回值：无
void Uart4Telemetry_UnlockConfig(void)
{
    (void)osMutexRelease(s_config_mutex);
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
