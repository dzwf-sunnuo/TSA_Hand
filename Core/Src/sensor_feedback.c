#include "sensor_feedback.h"
#include "Motor.h"
#include "Tactile_Sensor.h"
#include "adc.h"
#include "rs485.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define SENSOR_ADC_WORD_COUNT 60U
#define SENSOR_ADC_TIMEOUT_TICKS 2U
#define SENSOR_CYCLE_IO_BUDGET_MS 8U
#define SENSOR_FORCE_RETRY_MS 100U

extern uint16_t ADC_NativeValue[60]; // 普通SRAM，DMA可以访问
extern uint16_t ADC_Snapshot[60];    // 保留原始调试快照，只有冻结后才写入
extern osSemaphoreId_t Sem_ADC_Done;

// 成品缓存、传感器句柄及批次状态仅由CPU/中断访问，放入CCMRAM；DMA原始缓冲仍在普通SRAM。
__CCM_RAM_DATA static SensorFeedbackSnapshot_t s_snapshot;
__CCM_RAM_DATA static Tactile_Sensor_t s_sensors[SENSOR_FORCE_COUNT];
__CCM_RAM_DATA static uint32_t s_retry_tick[SENSOR_FORCE_COUNT];
__CCM_RAM_DATA static uint8_t s_retry_pending[SENSOR_FORCE_COUNT];
__CCM_RAM_DATA static volatile uint32_t s_adc_batch;
__CCM_RAM_DATA static volatile uint32_t s_adc_completed_batch;
__CCM_RAM_DATA static volatile uint32_t s_adc_complete_tick;
__CCM_RAM_DATA static volatile uint8_t s_adc_active;
__CCM_RAM_DATA static volatile uint8_t s_adc_success;

// SensorFeedback_IsFresh：判断有效样本是否仍在允许年龄范围内
// 参数：valid - 有效状态；update_ms - 成功时间；now_ms - 当前时间
// 返回值：true表示有效且年龄不超过50毫秒
bool SensorFeedback_IsFresh(uint8_t valid, uint32_t update_ms, uint32_t now_ms)
{
    return valid != 0U && (uint32_t)(now_ms - update_ms) <= SENSOR_MAX_AGE_MS;
}

// SensorFeedback_Init：初始化PA8/PD1/PD2对应的中指、无名指、小指
// 参数：无；须在TIM13启动后、RTOS调度前调用
// 返回值：无；所有反馈最初均无效
void SensorFeedback_Init(void)
{
    static GPIO_TypeDef *const ports[SENSOR_FORCE_COUNT] = {GPIOA, GPIOD, GPIOD};
    static const uint16_t pins[SENSOR_FORCE_COUNT] = {GPIO_PIN_8, GPIO_PIN_1, GPIO_PIN_2};
    static const char *const names[SENSOR_FORCE_COUNT] = {"Middle", "Ring", "Little"};
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    for (uint8_t i = 0U; i < SENSOR_FORCE_COUNT; i++) {
        (void)Tactile_Init(&s_sensors[i], &hspi3, ports[i], pins[i], names[i]);
        Tactile_RegisterSensor(&s_sensors[i]);
        s_retry_tick[i] = HAL_GetTick();
    }
}

// SensorFeedback_ReadSnapshot：一次复制完整缓存，避免跨批次拼接字段
// 参数：snapshot - 输出地址，空指针时不操作
// 返回值：无；不取得电机寄存器锁，不等待ADC或SPI
void SensorFeedback_ReadSnapshot(SensorFeedbackSnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    *snapshot = s_snapshot;
    uint16_t enabled = Reg[TACTILE_REG_ENABLE];
    taskEXIT_CRITICAL();
    for (uint8_t i = 0U; i < SENSOR_FORCE_COUNT; i++) {
        // 即使采集任务尚未发布下一批，关闭采集也会立即撤销反馈有效位。
        if ((enabled & (uint16_t)(1U << tactile_motor_indices[i])) == 0U) {
            snapshot->force[i].valid = 0U;
        }
    }
}

// SensorFeedback_AdcFinishedFromISR：冻结有限批次后再唤醒采集任务
// 参数：success - 满块成功为1，ADC或DMA错误为0
// 返回值：无；禁止在此进行滤波、浮点计算或HAL阻塞停止
void SensorFeedback_AdcFinishedFromISR(uint8_t success)
{
    // DMA正常模式满块后不再写内存，同时立即停ADC，避免后续转换触发溢出。
    CLEAR_BIT(hadc1.Instance->CR2, ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON);
    if (s_adc_active == 0U) {
        return;
    }
    s_adc_active = 0U;
    s_adc_success = success;
    s_adc_complete_tick = HAL_GetTick();
    s_adc_completed_batch = s_adc_batch;
    __DMB();
    if (Sem_ADC_Done != NULL) {
        (void)osSemaphoreRelease(Sem_ADC_Done);
    }
}

// SensorFeedback_StopAdc：硬件确实停止后才恢复ADC与本流DMA的HAL状态
// 参数：无；只操作ADC1和DMA2_Stream0，不重置与USART1共享的整个DMA2
// 返回值：true表示已冻结可重启，false表示100微秒内未停止
static bool SensorFeedback_StopAdc(void)
{
    DMA_HandleTypeDef *dma = hadc1.DMA_Handle;
    s_adc_active = 0U;
    CLEAR_BIT(hadc1.Instance->CR2, ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON);
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_OVR | ADC_IT_EOC);
    __HAL_DMA_DISABLE(dma);
    uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim13);
    while ((dma->Instance->CR & DMA_SxCR_EN) != 0U) {
        if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim13) - start) >= 100U) {
            return false; // 硬件仍活动时不改成READY，也不读取原始缓冲。
        }
    }
    __HAL_DMA_DISABLE_IT(dma, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(dma, DMA_IT_FE);
    __HAL_DMA_CLEAR_FLAG(dma, DMA_FLAG_FEIF0_4 | DMA_FLAG_DMEIF0_4 |
                        DMA_FLAG_TEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_STRT);
    // ADC1仅运行本批规则采集；旧HAL错误位不会由Start_DMA自动清除。
    dma->State = HAL_DMA_STATE_READY;
    dma->ErrorCode = HAL_DMA_ERROR_NONE;
    dma->Lock = HAL_UNLOCKED;
    hadc1.State = HAL_ADC_STATE_READY;
    hadc1.ErrorCode = HAL_ADC_ERROR_NONE;
    hadc1.Lock = HAL_UNLOCKED;
    return true;
}

// SensorFeedback_ReadAdc：启动并取得本轮稳定的四通道15点原始样本
// 参数：snapshot - 本轮任务私有结果，失败时保留旧数值但清除有效标志
// 返回值：无
static void SensorFeedback_ReadAdc(SensorFeedbackSnapshot_t *snapshot)
{
    snapshot->adc_valid = 0U;
    snapshot->angle_valid_mask = 0U;
    if (!SensorFeedback_StopAdc() || Sem_ADC_Done == NULL) {
        return;
    }
    // 停止旧批次、清除旧中断与旧通知后才分配新编号，杜绝提前接受上轮完成。
    __HAL_DMA_CLEAR_FLAG(hadc1.DMA_Handle, DMA_FLAG_FEIF0_4 | DMA_FLAG_DMEIF0_4 |
                         DMA_FLAG_TEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_STRT);
    while (osSemaphoreAcquire(Sem_ADC_Done, 0U) == osOK) {
    }
    uint32_t batch = ++s_adc_batch;
    s_adc_success = 0U;
    s_adc_active = 1U;
    // 完成中断会撤销DDS；每一新批次都重新允许四通道序列间的DMA请求。
    SET_BIT(hadc1.Instance->CR2, ADC_CR2_DDS);
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_NativeValue,
                          SENSOR_ADC_WORD_COUNT) != HAL_OK) {
        (void)SensorFeedback_StopAdc();
        return;
    }
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT); // 只需要满60点的完成通知
    osStatus_t wait_status = osSemaphoreAcquire(Sem_ADC_Done, SENSOR_ADC_TIMEOUT_TICKS);
    bool stopped = SensorFeedback_StopAdc();
    if (!stopped || wait_status != osOK || s_adc_completed_batch != batch || s_adc_success == 0U) {
        return;
    }
    memcpy(ADC_Snapshot, ADC_NativeValue, sizeof(uint16_t) * SENSOR_ADC_WORD_COUNT);
    for (uint8_t channel = 0U; channel < SENSOR_JOINT_COUNT; channel++) {
        uint32_t sum = 0U;
        for (uint8_t sample = 0U; sample < 15U; sample++) {
            sum += ADC_NativeValue[4U * sample + channel];
        }
        snapshot->adc_mean[channel] = (uint16_t)(sum / 15U);
        if (Motor_ConvertJointAngleTenths(channel, snapshot->adc_mean[channel],
                                          &snapshot->joint_angle_tenths[channel])) {
            snapshot->angle_valid_mask |= (uint8_t)(1U << channel);
        }
    }
    snapshot->adc_update_tick = s_adc_complete_tick;
    snapshot->adc_valid = 1U;
}

// SensorFeedback_Acquire：在10毫秒周期内采集ADC并轮询启用的三个力传感器
// 参数：cycle_start_ms - 本周期实际开始的HAL时刻
// 返回值：无；只在最后的短临界区发布缓存
void SensorFeedback_Acquire(uint32_t cycle_start_ms)
{
    SensorFeedbackSnapshot_t next;
    SensorFeedback_ReadSnapshot(&next);
    taskENTER_CRITICAL();
    uint16_t enabled = Reg[TACTILE_REG_ENABLE];
    taskEXIT_CRITICAL();
    SensorFeedback_ReadAdc(&next);
    for (uint8_t i = 0U; i < SENSOR_FORCE_COUNT; i++) {
        next.force[i].valid = 0U;
        uint32_t now_ms = HAL_GetTick();
        if ((enabled & (uint16_t)(1U << tactile_motor_indices[i])) == 0U) {
            s_retry_pending[i] = 0U; // 重新启用通道时立即尝试，不沿用先前的退避。
            continue;
        }
        // 为一次最长2毫秒的SPI事务留出余量；故障通道不拖延整轮到下一周期。
        if ((uint32_t)(now_ms - cycle_start_ms) >= SENSOR_CYCLE_IO_BUDGET_MS - 2U ||
            (s_retry_pending[i] != 0U &&
             (uint32_t)(now_ms - s_retry_tick[i]) < SENSOR_FORCE_RETRY_MS)) {
            continue;
        }
        HAL_StatusTypeDef status = Tactile_Update(&s_sensors[i]);
        if (status != HAL_OK || s_sensors[i].valid == 0U) {
            s_retry_tick[i] = HAL_GetTick();
            s_retry_pending[i] = 1U;
            continue;
        }
        s_retry_pending[i] = 0U;
        next.force[i].fx = s_sensors[i].fx;
        next.force[i].fy = s_sensors[i].fy;
        next.force[i].fz = s_sensors[i].fz;
        next.force[i].f_sum = s_sensors[i].f_sum;
        next.force[i].tangential_force = sqrtf(s_sensors[i].fx * s_sensors[i].fx +
                                               s_sensors[i].fy * s_sensors[i].fy);
        next.force[i].normal_force = fabsf(s_sensors[i].fz);
        next.force[i].last_update_tick = s_sensors[i].last_update_tick;
        next.force[i].valid = 1U;
    }
    next.acquisition_sequence++;
    taskENTER_CRITICAL();
    s_snapshot = next; // 固定长度复制；不在此做任何硬件等待、浮点运算或发送。
    taskEXIT_CRITICAL();
}
