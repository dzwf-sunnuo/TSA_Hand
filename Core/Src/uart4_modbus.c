#include "uart4_modbus.h"
#include "usart.h"
#include "rs485.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <string.h>

#define UART4_DMA_RX_SIZE 256U
#define UART4_RX_RING_SIZE 2048U
#define UART4_REQUEST_SIZE 100U
#define UART4_RX_WORK_BUDGET 256U
#define UART4_RX_RETRY_MS 100U

volatile Uart4ModbusStats_t uart4ModbusStats;
// DMA直接访问的缓冲必须留在普通SRAM；不能迁入CCMRAM。
static uint8_t s_dma_rx[UART4_DMA_RX_SIZE];
static uint8_t s_ring[UART4_RX_RING_SIZE];
static uint8_t s_request[UART4_REQUEST_SIZE];
static uint16_t s_request_length;
static volatile uint16_t s_ring_write;
static volatile uint16_t s_ring_read;
static uint16_t s_dma_position;
static volatile uint8_t s_overflow;
static volatile uint8_t s_rx_error;
static volatile uint8_t s_started;
static volatile uint32_t s_last_byte_ms;
static uint32_t s_last_retry_ms;
static osSemaphoreId_t s_wake;

// Uart4Modbus_ResetPending：原子放弃软件缓存及半帧，溢出后禁止拼接新旧数据。
// 参数：无；由控制任务调用。
// 返回值：无。
static void Uart4Modbus_ResetPending(void)
{
    taskENTER_CRITICAL();
    s_ring_read = s_ring_write;
    s_overflow = 0U;
    s_request_length = 0U;
    taskEXIT_CRITICAL();
}

// Uart4Modbus_RxDmaError：替代HAL默认同时结束收发的DMA错误回调。
// 参数：hdma - UART4的接收DMA句柄，仅用于HAL回调签名。
// 返回值：无，只请求任务恢复RX，绝不修改TX状态。
static void Uart4Modbus_RxDmaError(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    Uart4Modbus_OnRxError();
}

// Uart4Modbus_StartReceive：在RX空闲时启动循环DMA，保持半满和全满通知。
// 参数：无；由控制任务调用。
// 返回值：无，失败后隔100毫秒重试。
static void Uart4Modbus_StartReceive(void)
{
    uint32_t now = HAL_GetTick();
    if (s_started != 0U || (uint32_t)(now - s_last_retry_ms) < UART4_RX_RETRY_MS) return;
    s_last_retry_ms = now;
    taskENTER_CRITICAL();
    s_dma_position = 0U;
    // 启动期间短暂屏蔽中断，防止HAL尚未完成配置就交付首个接收事件。
    s_started = 1U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_dma_rx, sizeof(s_dma_rx)) != HAL_OK) {
        s_started = 0U;
        uart4ModbusStats.start_failures++;
    } else {
        // HAL默认UART_DMAError会同时结束RX和TX，改为只处理当前接收方向。
        huart4.hdmarx->XferErrorCallback = Uart4Modbus_RxDmaError;
    }
    taskEXIT_CRITICAL();
}

// Uart4Modbus_Init：准备缓存与唤醒句柄，第一次立即尝试启动循环DMA。
// 参数：wake - 统一控制任务的信号量。
// 返回值：无。
void Uart4Modbus_Init(osSemaphoreId_t wake)
{
    s_wake = wake;
    s_last_retry_ms = HAL_GetTick() - UART4_RX_RETRY_MS;
    Uart4Modbus_ResetPending();
    Uart4Modbus_StartReceive();
}

// Uart4Modbus_CopyDmaBytes：在中断内复制一段新增字节，软件缓存满则标记溢出。
// 参数：first - DMA缓存起始下标；count - 本次连续字节数。
// 返回值：无；溢出后等待任务清空缓存，不覆盖尚未消费的数据。
static void Uart4Modbus_CopyDmaBytes(uint16_t first, uint16_t count)
{
    uart4ModbusStats.rx_bytes += count;
    if (s_overflow != 0U) return;
    uint16_t write = s_ring_write;
    for (uint16_t i = 0U; i < count; i++) {
        uint16_t next = (uint16_t)((write + 1U) % UART4_RX_RING_SIZE);
        if (next == s_ring_read) {
            s_overflow = 1U;
            uart4ModbusStats.ring_overflows++;
            break;
        }
        s_ring[write] = s_dma_rx[first + i];
        write = next;
    }
    // 先发布数据，再发布写游标，任务不会看到尚未写好的字节。
    __DMB();
    s_ring_write = write;
}

// Uart4Modbus_OnRxEvent：根据实际DMA游标复制新增区域，IDLE不会终止或清空半帧。
// 参数：无；UART4与RX DMA中断保持相同抢占优先级，避免重入修改游标。
// 返回值：无，只搬运数据并唤醒任务。
void Uart4Modbus_OnRxEvent(void)
{
    if (s_started == 0U || s_rx_error != 0U) return;
    // 不直接使用回调Size，避免延迟到达的HT事件回报旧位置而造成重复复制。
    uint16_t position = (uint16_t)(UART4_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx));
    if (position == UART4_DMA_RX_SIZE) position = 0U;
    if (position == s_dma_position) return;
    if (position > s_dma_position) {
        Uart4Modbus_CopyDmaBytes(s_dma_position, position - s_dma_position);
    } else {
        Uart4Modbus_CopyDmaBytes(s_dma_position, UART4_DMA_RX_SIZE - s_dma_position);
        Uart4Modbus_CopyDmaBytes(0U, position);
    }
    s_dma_position = position;
    s_last_byte_ms = HAL_GetTick();
    if (s_wake != NULL) (void)osSemaphoreRelease(s_wake);
}

// Uart4Modbus_OnRxError：记录接收故障并请求任务恢复，禁止在中断内停止TX。
// 参数：无。
// 返回值：无。
void Uart4Modbus_OnRxError(void)
{
    s_rx_error = 1U;
    uart4ModbusStats.rx_errors++;
    if (s_wake != NULL) (void)osSemaphoreRelease(s_wake);
}

// Uart4Modbus_DropPrefix：移除已消费或已判错的前缀，保留后续完整帧和残片。
// 参数：count - 待移除字节数。
// 返回值：无。
static void Uart4Modbus_DropPrefix(uint16_t count)
{
    s_request_length -= count;
    memmove(s_request, &s_request[count], s_request_length);
}

// Uart4Modbus_ParsePending：扫描合法地址、功能码、长度和CRC，允许跳过错误半帧。
// 参数：无；解析的是任务私有请求缓存。
// 返回值：true表示消费了一条完整请求，可继续处理缓存中的下一条。
static bool Uart4Modbus_ParsePending(void)
{
    uint16_t pending = s_request_length;
    for (uint16_t offset = 0U; offset < s_request_length; offset++) {
        uint16_t available = s_request_length - offset;
        uint8_t *data = &s_request[offset];
        if (data[0] != modbus2.myadd && data[0] != 0U) continue;
        if (available < 2U) {
            if (offset < pending) pending = offset;
            continue;
        }
        uint16_t length;
        if (data[1] == 0x03U || data[1] == 0x06U) {
            length = 8U;
        } else if (data[1] == 0x10U) {
            if (available < 7U) {
                if (offset < pending) pending = offset;
                continue;
            }
            uint16_t count = ((uint16_t)data[4] << 8) | data[5];
            if (count == 0U || count > 45U || data[6] != 2U * count) continue;
            length = (uint16_t)(9U + data[6]);
        } else {
            continue;
        }
        if (available < length) {
            if (offset < pending) pending = offset;
            continue;
        }
        if (!Modbus_IsRequestFrame(data, length)) {
            uart4ModbusStats.crc_failures++;
            continue;
        }
        uart4ModbusStats.valid_frames++;
        uart4ModbusStats.discarded_bytes += offset;
        // false使正常应答和异常应答都被禁止，业务仍使用USART1相同的入口。
        if (Modbus_ProcessRequest(data, length, false)) uart4ModbusStats.accepted++;
        else uart4ModbusStats.rejected++;
        Uart4Modbus_DropPrefix(offset + length);
        return true;
    }
    if (pending != 0U) {
        uart4ModbusStats.discarded_bytes += pending;
        Uart4Modbus_DropPrefix(pending);
    }
    return false;
}

// Uart4Modbus_Process：在统一任务中恢复RX、检查残片超时并限量处理缓存。
// 参数：无；每轮最多取256字节，UART1和UART4不会并发执行寄存器业务。
// 返回值：无。
void Uart4Modbus_Process(void)
{
    if (s_rx_error != 0U) {
        s_started = 0U;
        // 只停止接收方向，UART4遥测的发送DMA继续运行。
        if (HAL_UART_AbortReceive(&huart4) != HAL_OK ||
            (huart4.hdmarx->Instance->CR & DMA_SxCR_EN) != 0U) return;
        Uart4Modbus_ResetPending();
        s_rx_error = 0U;
        s_last_retry_ms = HAL_GetTick() - UART4_RX_RETRY_MS;
    }
    Uart4Modbus_StartReceive();
    if (s_request_length != 0U && s_ring_read == s_ring_write &&
        (uint32_t)(HAL_GetTick() - s_last_byte_ms) >= UART4_MODBUS_FRAGMENT_TIMEOUT_MS) {
        uart4ModbusStats.fragment_timeouts++;
        uart4ModbusStats.discarded_bytes += s_request_length;
        s_request_length = 0U;
    }
    for (uint16_t consumed = 0U; consumed < UART4_RX_WORK_BUDGET; consumed++) {
        if (s_overflow != 0U) Uart4Modbus_ResetPending();
        uint16_t read = s_ring_read;
        if (read == s_ring_write) break;
        uint8_t value = s_ring[read];
        __DMB();
        s_ring_read = (uint16_t)((read + 1U) % UART4_RX_RING_SIZE);
        if (s_request_length == sizeof(s_request)) {
            uart4ModbusStats.discarded_bytes++;
            Uart4Modbus_DropPrefix(1U);
        }
        s_request[s_request_length++] = value;
        while (Uart4Modbus_ParsePending()) {
            // 只有完整且CRC合法的请求才会执行，未完成的数据留在缓存。
        }
    }
    if (s_ring_read != s_ring_write && s_wake != NULL) (void)osSemaphoreRelease(s_wake);
}
