#ifndef UART4_MODBUS_H
#define UART4_MODBUS_H

#include "cmsis_os2.h"
#include <stdint.h>

// 蓝牙残片超时只用于清理未完成请求，不用于判断正常帧结束。
#define UART4_MODBUS_FRAGMENT_TIMEOUT_MS 500U

// Uart4ModbusStats_t：记录接收、组帧和执行结果，便于调试时区分链路问题。
// 参数：所有字段为累计计数；accepted表示公共参数校验及处理成功，不表示机械到位。
// 返回值：统计结构类型，不通过UART4输出额外文本。
typedef struct {
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t accepted;
    uint32_t rejected;
    uint32_t discarded_bytes;
    uint32_t crc_failures;
    uint32_t ring_overflows;
    uint32_t fragment_timeouts;
    uint32_t rx_errors;
    uint32_t start_failures;
} Uart4ModbusStats_t;

extern volatile Uart4ModbusStats_t uart4ModbusStats;

// Uart4Modbus_Init：初始化独立RX缓存并启动循环DMA。
// 参数：wake - 与USART1共用的控制任务唤醒信号量，须在内核启动后调用。
// 返回值：无；接收启动失败会由Process重试。
void Uart4Modbus_Init(osSemaphoreId_t wake);
// Uart4Modbus_Process：恢复接收、清理残片并在统一控制任务中组帧执行。
// 参数：无；只能由ModbusPC任务调用，不得在中断中调用。
// 返回值：无；每次最多消费256字节，避免饿死USART1。
void Uart4Modbus_Process(void);
// Uart4Modbus_OnRxEvent：把DMA新增字节交给软件环形缓存，不进行协议解析。
// 参数：无；仅由UART4的IDLE、DMA半满和全满回调调用。
// 返回值：无。
void Uart4Modbus_OnRxEvent(void);
// Uart4Modbus_OnRxError：标记接收错误，交给任务只恢复RX。
// 参数：无；仅在UART4接收或接收DMA错误时调用。
// 返回值：无，不中止TX遥测。
void Uart4Modbus_OnRxError(void);

#endif
