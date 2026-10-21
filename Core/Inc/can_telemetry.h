#ifndef CAN_TELEMETRY_H
#define CAN_TELEMETRY_H

#include "Motor.h"
#include <stdbool.h>

#define CAN_TELEMETRY_ANGLE_ID 0x20U
#define CAN_TELEMETRY_FORCE_ID 0x21U

// CanTelemetryStats_t：记录上报异常，供调试器观察，不占用额外协议帧
// 参数：无；计数为累计次数，last_esr保存最近观察到的CAN错误寄存器
// 返回值：无
typedef struct {
    uint32_t init_failures;
    uint32_t skipped_groups;
    uint32_t tx_failures;
    uint32_t aborted_frames;
    uint32_t bus_off_count;
    uint32_t last_esr;
} CanTelemetryStats_t;

extern volatile CanTelemetryStats_t canTelemetryStats;

// CanTelemetry_Init：以现有875 kbit/s参数启动只发送反馈的CAN1
// 参数：无
// 返回值：HAL_OK表示启动成功，失败由上报任务重试，不停止其他任务
HAL_StatusTypeDef CanTelemetry_Init(void);

// CanTelemetry_Encode：将同一份缓存快照编码为8字节角度与8字节力，两帧D7共用序号
// 参数：snapshot - 缓存；now_tick - HAL毫秒时刻；sequence - 共同序号；两个data为输出
// 返回值：true表示角度有效可发，力的有效位独立放在force_data[6]
bool CanTelemetry_Encode(const MotorTelemetrySnapshot_t *snapshot, uint32_t now_tick,
                         uint8_t sequence, uint8_t angle_data[8], uint8_t force_data[8]);

// CanTelemetry_Process：从缓存取一组反馈并非阻塞提交邮箱
// 参数：now_tick - 当前HAL毫秒时刻
// 返回值：无；邮箱满、断线或数据无效均不等待重传
void CanTelemetry_Process(uint32_t now_tick);

// CanTelemetry_NotifyConfigChanged：唤醒上报任务处理有效的MODBUS配置变化
// 参数：无
// 返回值：无；可在任务上下文调用，不等待CAN发送
void CanTelemetry_NotifyConfigChanged(void);

// vCanTelemetryTask：维护CAN故障和MODBUS配置的公共上报频率
// 参数：argument - 未使用
// 返回值：无，任务持续运行
void vCanTelemetryTask(void *argument);

#endif
