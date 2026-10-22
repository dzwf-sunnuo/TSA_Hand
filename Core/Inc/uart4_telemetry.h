#ifndef UART4_TELEMETRY_H
#define UART4_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define UART4_TELEMETRY_FRAME_SIZE 25U
#define UART4_TELEMETRY_INVALID_VALUE 0xFFFFU

// Uart4TelemetryStats_t：保存上报结果，不向业务串口打印调试文本
// 参数：各字段为累计帧数或错误次数
// 返回值：无
typedef struct {
    uint32_t sent_frames;
    uint32_t busy_skips;
    uint32_t tx_errors;
} Uart4TelemetryStats_t;

extern volatile Uart4TelemetryStats_t uart4TelemetryStats;

// Uart4Telemetry_Init：创建读取反馈缓存的独立上报任务
// 参数：无
// 返回值：true表示任务创建成功
bool Uart4Telemetry_Init(void);

// Uart4Telemetry_NotifyConfigChanged：配置提交后唤醒任务处理启停或改频
// 参数：无，可在任务上下文调用
// 返回值：无
void Uart4Telemetry_NotifyConfigChanged(void);

// Uart4Telemetry_OnTxComplete：释放已完整发送的UART4缓冲区
// 参数：无，仅由UART4发送完成回调调用
// 返回值：无
void Uart4Telemetry_OnTxComplete(void);

// Uart4Telemetry_OnTxError：标记发送异常，由上报任务恢复HAL发送状态
// 参数：无，仅由UART4错误回调调用
// 返回值：无
void Uart4Telemetry_OnTxError(void);

#endif
