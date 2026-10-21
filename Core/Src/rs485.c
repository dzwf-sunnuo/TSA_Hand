#include "rs485.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "can_telemetry.h"

MODBUS modbus;
uint16_t Reg[100] __attribute__((section(".ccmram"))) = {0};
volatile uint8_t pid_params_dirty = 0;  // Modbus 写 Reg[5..13] 时置 1
volatile uint8_t admittance_mode_restart = 0; // bit0=模式6，bit1=模式7
volatile uint8_t experiment_event = 0; // 1=实验开始，2=实验停止
volatile uint8_t experiment_mode = 0;
volatile uint8_t experiment_output_limit = 0;

#define EXPERIMENT_TARGET_TURNS 60U
#define EXPERIMENT_RETURN_COMMAND 0x0064U

// UpdateAdmittanceRestartFlag：检测模式寄存器写入并生成导纳重新启动事件
// 参数：reg_value - 本次写入Reg[4]的完整16位值
// 返回值：无
static void UpdateAdmittanceRestartFlag(uint16_t reg_value)
{
    if (reg_value == 0x0601U) {
        admittance_mode_restart |= 0x01U;
    } else if (reg_value == 0x0701U) {
        admittance_mode_restart |= 0x02U;
    }
}

// IsExperimentOutputLimit：判断输出限幅是否属于五档减速比实验
// 参数：output_limit - Reg[0]低字节中的输出限幅百分比
// 返回值：uint8_t - 属于20、40、60、80、100时返回1，否则返回0
static uint8_t IsExperimentOutputLimit(uint8_t output_limit)
{
    return (uint8_t)(output_limit == 20U || output_limit == 40U ||
                     output_limit == 60U || output_limit == 80U ||
                     output_limit == 100U);
}

// UpdateExperimentEvent：根据当前电机0目标和模式生成减速比实验事件
// 参数：无
// 返回值：无
static void UpdateExperimentEvent(void)
{
    uint8_t mode = (uint8_t)(Reg[4] >> 8);
    uint8_t enable = (uint8_t)(Reg[4] & 0xFFU);
    uint8_t target_turns = (uint8_t)(Reg[0] >> 8);
    uint8_t output_limit = (uint8_t)(Reg[0] & 0xFFU);

    if ((mode == 1U || mode == 3U) && enable != 0U &&
        Reg[0] == EXPERIMENT_RETURN_COMMAND) {
        experiment_event = 2U;
    } else if ((mode == 1U || mode == 3U) && enable != 0U &&
               target_turns == EXPERIMENT_TARGET_TURNS &&
               IsExperimentOutputLimit(output_limit) != 0U) {
        experiment_mode = mode;
        experiment_output_limit = output_limit;
        experiment_event = 1U;
    } else if (mode == 4U || enable == 0U) {
        experiment_event = 2U;
    }
}

// LED 闪烁状态 (RS485 收发时双闪)
static int led_blink_cnt = 0;  // 剩余闪烁次数 (0=空闲)

// LED 闪烁定时器回调: 每 100ms 翻转 PE2, 4 次 = 2 闪 (400ms)
void LedBlinkTimerCallback(void *argument)
{
    (void)argument;
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
    if (++led_blink_cnt < 2) {
        osTimerStart(ledBlinkTimerHandle, LED_BLINK_PERIOD_MS);
    } else {
        led_blink_cnt = 0;
    }
}

/**
 * @brief 485 串口发送单字节 (保留原 API)
 */
void Modbus_Send_Byte(uint8_t ch)
{
    // 非阻塞发送，等待发送完成信号量替代 busy-wait
    if (HAL_UART_Transmit_IT(&huart1, &ch, 1) == HAL_OK) {
        if (uartTxSemHandle != NULL) {
            (void)osSemaphoreAcquire(uartTxSemHandle, 100); // 最长等待 100ms
        }
    }
}

// Modbus_Init：初始化一号板协议、两路采集使能与默认20Hz的CAN自动上报
// 参数：无
// 返回值：无
void Modbus_Init(void)
{
    memset(&modbus, 0, sizeof(MODBUS));
    
    // 初始化寄存器默认值
    Reg[0] = 0x0000; 
    Reg[1] = 0x0000; 
    Reg[2] = 0x0000; 
    Reg[3] = 0x0000; 
    Reg[4] = 0x0000; 

    // 拇指与食指两路采集、CAN上报均默认开启，电机运动使能仍由Reg[4]独立控制。
    Reg[TACTILE_REG_ENABLE] = TACTILE_DEFAULT_ENABLE;
    Reg[CAN_REPORT_REG_ENABLE] = CAN_REPORT_DEFAULT_ENABLE;
    Reg[CAN_REPORT_REG_FREQUENCY] = CAN_REPORT_DEFAULT_FREQUENCY;

    modbus.myadd = RS485_SLAVE_ADDR;    
    
    
    // 强制清除可能存在的错误标志
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    // 开启 DMA 空闲中断接收 (DMA + IDLE)
    // 第一次调用此函数会开启 DMA 传输，并使能串口的空闲中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus.rcbuf, MODBUS_BUF_SIZE);
}

/**
 * @brief UART 接收事件回调 (处理 IDLE 或 DMA 传输完成)
 * @param huart 串口句柄
 * @param Size 实际接收到的字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        MODBUS_FrameTypeDef frame;

        if (Size > MODBUS_BUF_SIZE)
        {
            Size = MODBUS_BUF_SIZE;
        }

        frame.length = Size;
        memcpy(frame.data, modbus.rcbuf, Size);

        if (modbusRxQueueHandle != NULL)
        {
            (void)osMessageQueuePut(modbusRxQueueHandle, &frame, 0U, 0U);
        }

        // RS485 收到数据 → 触发 LED 双闪 (ISR 内必须用 FromISR 版本)
        led_blink_cnt = 0;
        if (ledBlinkTimerHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTimerStartFromISR((TimerHandle_t)ledBlinkTimerHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        // 2. 重新开启下一次接收 (DMA 模式设为 NORMAL 时需要重新开启)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus.rcbuf, MODBUS_BUF_SIZE);
    }
}

/**
 * @brief 串口接收完成回调函数 (旧的逐字节方式已停用，若有其他逻辑可保留，此处注释掉)
 */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // DMA 模式下不再进入此回调
}
*/

/**
 * @brief 串口错误回调函数 (核心修复：防止 ORE 溢出导致接收挂死)
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint32_t isrflags = READ_REG(huart->Instance->SR);
        if ((isrflags & UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(huart); 
            // 重新开启接收
            HAL_UARTEx_ReceiveToIdle_DMA(huart, modbus.rcbuf, MODBUS_BUF_SIZE);
        }
    }
}

/**
 * @brief UART 发送完成回调（由 HAL 调用）
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (uartTxSemHandle != NULL) {
            (void)osSemaphoreRelease(uartTxSemHandle);
        }
        // RS485 发送完成 → 触发 LED 双闪 (ISR 内必须用 FromISR 版本)
        led_blink_cnt = 0;
        if (ledBlinkTimerHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTimerStartFromISR((TimerHandle_t)ledBlinkTimerHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief 串口空闲中断回调处理函数 (旧回调，由 stm32f4xx_it.c 调用)
 * @note  在使用 HAL_UARTEx_ReceiveToIdle_DMA 时，HAL 内部已处理 IDLE 标志，
 *        此函数现在变为可选。
 */
void RS485_UART_IDLE_CallBack(void)
{
    // 如果使用了 HAL_UARTEx_RxEventCallback，这里通常可以为空，
    // 因为 HAL_UART_IRQHandler 会自动处理 IDLE 标志并触发回调。
}

/**
 * @brief Modbus 事件处理逻辑
 */
void Modbus_Event(const MODBUS_FrameTypeDef *frame)
{
    uint16_t crc, rccrc;

    if (frame == NULL || frame->length < 4U)
    {
        return;
    }

    crc = Modbus_CRC16(frame->data, frame->length - 2U);
    rccrc = (uint16_t)(frame->data[frame->length - 2U] * 256U + frame->data[frame->length - 1U]);
    
    if (crc == rccrc)
    {
        if (frame->data[0] == modbus.myadd || frame->data[0] == 0x00) 
        {
            switch (frame->data[1])
            {
                case 3:  Modbus_Func3(frame->data, frame->length);  break;
                case 6:  Modbus_Func6(frame->data, frame->length);  break;
                case 16: Modbus_Func16(frame->data, frame->length); break;
                default: break;
            }
        }
    }
}

// Modbus_GetCanReportConfig：原子读取公共开关和频率，避免多寄存器更新中途取值
// 参数：enabled - 开关输出地址；frequency - 频率输出地址
// 返回值：无；不等待电机互斥锁
void Modbus_GetCanReportConfig(uint16_t *enabled, uint16_t *frequency)
{
    if (enabled == NULL || frequency == NULL) return;
    taskENTER_CRITICAL();
    *enabled = Reg[CAN_REPORT_REG_ENABLE] == 1U ? 1U : 0U;
    *frequency = Reg[CAN_REPORT_REG_FREQUENCY];
    taskEXIT_CRITICAL();
    if (*frequency < CAN_REPORT_MIN_FREQUENCY || *frequency > CAN_REPORT_MAX_FREQUENCY) {
        *frequency = CAN_REPORT_DEFAULT_FREQUENCY;
    }
}

// Modbus_CanValueValid：仅校验新增寄存器，保留其他寄存器原有值域
// 参数：address - 寄存器地址；value - 准备写入的值
// 返回值：1表示允许，0表示新增配置值非法
static uint8_t Modbus_CanValueValid(uint16_t address, uint16_t value)
{
    if (address == CAN_REPORT_REG_ENABLE) return value <= 1U;
    if (address == CAN_REPORT_REG_FREQUENCY) {
        return value >= CAN_REPORT_MIN_FREQUENCY && value <= CAN_REPORT_MAX_FREQUENCY;
    }
    return 1U;
}

// Modbus_SendResponse：沿用项目现有CRC编码和USART1发送完成通知
// 参数：body_length - 已写入sendbuf的响应正文长度，不含CRC
// 返回值：无；等待串口时不持有电机寄存器锁或临界区
static void Modbus_SendResponse(uint16_t body_length)
{
    uint16_t crc = Modbus_CRC16(modbus.sendbuf, body_length);
    modbus.sendbuf[body_length++] = (uint8_t)(crc >> 8);
    modbus.sendbuf[body_length++] = (uint8_t)crc;
    if (uartTxSemHandle != NULL) {
        while (osSemaphoreAcquire(uartTxSemHandle, 0U) == osOK) {
        }
    }
    if (HAL_UART_Transmit_IT(&huart1, modbus.sendbuf, body_length) == HAL_OK &&
        uartTxSemHandle != NULL) {
        (void)osSemaphoreAcquire(uartTxSemHandle, 100U);
    }
}

// Modbus_SendException：对单播非法地址或非法值返回异常，广播保持静默
// 参数：buffer - 原请求；exception - 0x02非法地址或0x03非法数据值
// 返回值：无
static void Modbus_SendException(const uint8_t *buffer, uint8_t exception)
{
    if (buffer[0] == 0U) return;
    modbus.sendbuf[0] = modbus.myadd;
    modbus.sendbuf[1] = buffer[1] | 0x80U;
    modbus.sendbuf[2] = exception;
    Modbus_SendResponse(3U);
}

// Modbus_AfterWrite：在原子写入完成后处理PID、模式与实验事件
// 参数：address - 起始地址；count - 寄存器数量
// 返回值：无；调用者仍在短临界区中，函数不等待任何硬件
static void Modbus_AfterWrite(uint16_t address, uint16_t count)
{
    uint16_t end = address + count;
    if (address <= PID_REG_KD_SPD && end > PID_REG_KP_POS) pid_params_dirty = 1U;
    if (address <= 4U && end > 4U) UpdateAdmittanceRestartFlag(Reg[4]);
    if (address == 0U || (address <= 4U && end > 4U)) UpdateExperimentEvent();
}

// Modbus_Func3：读取保持寄存器，并限制响应长度以保护200字节发送缓冲
// 参数：buffer - 完整且CRC正确的请求；length - 实际帧长
// 返回值：无；广播读取不响应
void Modbus_Func3(const uint8_t *buffer, uint16_t length)
{
    if (buffer == NULL || length != 8U || buffer[0] == 0U) return;
    uint16_t address = (uint16_t)((buffer[2] << 8) | buffer[3]);
    uint16_t count = (uint16_t)((buffer[4] << 8) | buffer[5]);
    if (count == 0U || count > (MODBUS_BUF_SIZE - 5U) / 2U) {
        Modbus_SendException(buffer, 0x03U);
        return;
    }
    if (address >= 100U || (uint32_t)address + count > 100U) {
        Modbus_SendException(buffer, 0x02U);
        return;
    }
    modbus.sendbuf[0] = modbus.myadd;
    modbus.sendbuf[1] = 0x03U;
    modbus.sendbuf[2] = (uint8_t)(2U * count);
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < count; i++) {
        uint16_t value = Reg[address + i];
        modbus.sendbuf[3U + 2U * i] = (uint8_t)(value >> 8);
        modbus.sendbuf[4U + 2U * i] = (uint8_t)value;
    }
    taskEXIT_CRITICAL();
    Modbus_SendResponse((uint16_t)(3U + 2U * count));
}

// Modbus_Func6：校验后原子写入单寄存器，实际配置变化才通知CAN任务
// 参数：buffer - 完整且CRC正确的请求；length - 实际帧长
// 返回值：无；非法写入保持旧值，广播合法写入但不回包
void Modbus_Func6(const uint8_t *buffer, uint16_t length)
{
    if (buffer == NULL || length != 8U) return;
    uint16_t address = (uint16_t)((buffer[2] << 8) | buffer[3]);
    uint16_t value = (uint16_t)((buffer[4] << 8) | buffer[5]);
    if (address >= 100U) {
        Modbus_SendException(buffer, 0x02U);
        return;
    }
    if (Modbus_CanValueValid(address, value) == 0U) {
        Modbus_SendException(buffer, 0x03U);
        return;
    }
    taskENTER_CRITICAL();
    uint8_t changed = (address == CAN_REPORT_REG_ENABLE || address == CAN_REPORT_REG_FREQUENCY) &&
                       Reg[address] != value;
    Reg[address] = value;
    Modbus_AfterWrite(address, 1U);
    taskEXIT_CRITICAL();
    if (changed != 0U) CanTelemetry_NotifyConfigChanged();
    if (buffer[0] != 0U) {
        memcpy(modbus.sendbuf, buffer, 6U);
        Modbus_SendResponse(6U);
    }
}

// Modbus_Func16：完整校验多寄存器请求后一次提交，拒绝半成功状态
// 参数：buffer - 完整且CRC正确的请求；length - 实际帧长
// 返回值：无；所有格式和值校验在写入和模式副作用之前完成
void Modbus_Func16(const uint8_t *buffer, uint16_t length)
{
    if (buffer == NULL || length < 9U) return;
    uint16_t address = (uint16_t)((buffer[2] << 8) | buffer[3]);
    uint16_t count = (uint16_t)((buffer[4] << 8) | buffer[5]);
    if (count == 0U || count > (MODBUS_BUF_SIZE - 9U) / 2U ||
        buffer[6] != 2U * count || length != 9U + 2U * count) {
        Modbus_SendException(buffer, 0x03U);
        return;
    }
    if (address >= 100U || (uint32_t)address + count > 100U) {
        Modbus_SendException(buffer, 0x02U);
        return;
    }
    for (uint16_t i = 0U; i < count; i++) {
        uint16_t value = (uint16_t)((buffer[7U + 2U * i] << 8) | buffer[8U + 2U * i]);
        if (Modbus_CanValueValid(address + i, value) == 0U) {
            Modbus_SendException(buffer, 0x03U);
            return;
        }
    }
    uint8_t changed = 0U;
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < count; i++) {
        uint16_t current = address + i;
        uint16_t value = (uint16_t)((buffer[7U + 2U * i] << 8) | buffer[8U + 2U * i]);
        if ((current == CAN_REPORT_REG_ENABLE || current == CAN_REPORT_REG_FREQUENCY) &&
            Reg[current] != value) changed = 1U;
        Reg[current] = value;
    }
    Modbus_AfterWrite(address, count);
    taskEXIT_CRITICAL();
    if (changed != 0U) CanTelemetry_NotifyConfigChanged();
    if (buffer[0] != 0U) {
        modbus.sendbuf[0] = modbus.myadd;
        modbus.sendbuf[1] = 0x10U;
        memcpy(&modbus.sendbuf[2], &buffer[2], 4U);
        Modbus_SendResponse(6U);
    }
}
