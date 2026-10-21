#ifndef SENSOR_FEEDBACK_H
#define SENSOR_FEEDBACK_H

#include "main.h"
#include <stdbool.h>

#define SENSOR_JOINT_COUNT 4U
#define SENSOR_FORCE_COUNT 3U
#define SENSOR_ACQUISITION_HZ 100U
#define SENSOR_MAX_AGE_MS 50U

// SensorForceFeedback_t：保存单指最新力值、成功时间和本轮有效状态
// 参数：无；力单位为牛顿，时间使用HAL_GetTick毫秒时基
// 返回值：无
typedef struct {
    float fx, fy, fz, f_sum;
    float tangential_force, normal_force;
    uint32_t last_update_tick;
    uint8_t valid;
} SensorForceFeedback_t;

// SensorFeedbackSnapshot_t：供控制和CAN共同读取的完整采集缓存
// 参数：无；关节下标0～3，力下标0～2对应motor1～3
// 返回值：无
typedef struct {
    uint16_t adc_mean[SENSOR_JOINT_COUNT];
    uint16_t joint_angle_tenths[SENSOR_JOINT_COUNT];
    SensorForceFeedback_t force[SENSOR_FORCE_COUNT];
    uint32_t adc_update_tick;
    uint32_t acquisition_sequence;
    uint8_t adc_valid;
    uint8_t angle_valid_mask;
} SensorFeedbackSnapshot_t;

// SensorFeedback_Init：在TIM13启动后初始化三路传感器和无效缓存
// 参数：无
// 返回值：无；校准发送成功不会伪装成有效力数据
void SensorFeedback_Init(void);

// SensorFeedback_Acquire：执行一轮有限长度ADC采集和SPI轮询
// 参数：cycle_start_ms - 本轮开始的HAL时刻，限制整轮时间预算
// 返回值：无；结果以完整快照一次发布
void SensorFeedback_Acquire(uint32_t cycle_start_ms);

// SensorFeedback_ReadSnapshot：在短临界区复制成品缓存，不等待新采样
// 参数：snapshot - 输出地址
// 返回值：无
void SensorFeedback_ReadSnapshot(SensorFeedbackSnapshot_t *snapshot);

// SensorFeedback_AdcFinishedFromISR：结束本批ADC活动并唤醒任务
// 参数：success - 1表示满块完成，0表示DMA或ADC错误
// 返回值：无；只允许ADC1相关中断调用
void SensorFeedback_AdcFinishedFromISR(uint8_t success);

// SensorFeedback_IsFresh：按无符号时间差判断缓存是否仍可使用
// 参数：valid - 有效标志；update_ms - 成功时刻；now_ms - 当前时刻
// 返回值：true表示有效且未超过50毫秒
bool SensorFeedback_IsFresh(uint8_t valid, uint32_t update_ms, uint32_t now_ms);

#endif
