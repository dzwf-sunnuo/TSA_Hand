#include "rs485.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "timers.h"

MODBUS modbus;
uint16_t Reg[100] __attribute__((section(".ccmram"))) = {0};
volatile uint8_t pid_params_dirty = 0;  // Modbus 写 Reg[5..13] 时置 1
volatile uint8_t admittance_mode_restart = 0; // bit0=模式6，bit1=模式7
volatile uint8_t experiment_event = 0; // 1=实验开始，2=实验停止
volatile uint8_t experiment_mode = 0;
volatile uint8_t experiment_output_limit = 0;

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

// UpdateExperimentEvent：根据模式寄存器写入生成减速比实验事件
// 参数：reg_value - 本次写入Reg[4]的完整16位值
// 返回值：无
static void UpdateExperimentEvent(uint16_t reg_value)
{
    uint8_t mode = (uint8_t)(reg_value >> 8);
    uint8_t enable = (uint8_t)(reg_value & 0xFFU);

    if ((mode == 1U || mode == 3U) && enable != 0U) {
        experiment_mode = mode;
        experiment_output_limit = (uint8_t)(Reg[0] & 0xFFU);
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

/**
 * @brief Modbus 初始化
 */
void Modbus_Init()
{
    memset(&modbus, 0, sizeof(MODBUS));
    
    // 初始化寄存器默认值
    Reg[0] = 0x0000; 
    Reg[1] = 0x0000; 
    Reg[2] = 0x0000; 
    Reg[3] = 0x0000; 
    Reg[4] = 0x0000; 

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

/**
 * @brief 03 功能码处理：读取寄存器
 */
void Modbus_Func3(const uint8_t *buffer, uint16_t length)
{
    uint16_t Regadd, Reglen, crc;
    uint16_t i, send_len = 0;

    if (buffer == NULL || length < 8U)
    {
        return;
    }

    Regadd = (uint16_t)(buffer[2] * 256U + buffer[3]);
    Reglen = (uint16_t)(buffer[4] * 256U + buffer[5]);

    if (Regadd + Reglen > 100) return;

    modbus.sendbuf[send_len++] = modbus.myadd;
    modbus.sendbuf[send_len++] = 0x03;
    modbus.sendbuf[send_len++] = (uint8_t)(Reglen * 2);

    for (i = 0; i < Reglen; i++)
    {
        modbus.sendbuf[send_len++] = Reg[Regadd + i] >> 8;
        modbus.sendbuf[send_len++] = Reg[Regadd + i] & 0xFF;
    }

    crc = Modbus_CRC16(modbus.sendbuf, send_len);
    modbus.sendbuf[send_len++] = crc >> 8;
    modbus.sendbuf[send_len++] = crc & 0xFF;

    if (HAL_UART_Transmit_IT(&huart1, modbus.sendbuf, send_len) == HAL_OK) {
        if (uartTxSemHandle != NULL) {
            (void)osSemaphoreAcquire(uartTxSemHandle, 100); // 等待完成
        }
    }
}

/**
 * @brief 06 功能码处理：写单个寄存器
 */
void Modbus_Func6(const uint8_t *buffer, uint16_t length)
{
    uint16_t Regadd, val, crc;
    uint16_t send_len = 0;

    if (buffer == NULL || length < 8U)
    {
        return;
    }

    Regadd = (uint16_t)(buffer[2] * 256U + buffer[3]);
    val = (uint16_t)(buffer[4] * 256U + buffer[5]);

    if (Regadd >= 100) return;

    Reg[Regadd] = val;
    if (Regadd >= PID_REG_KP_POS && Regadd <= PID_REG_KD_SPD) pid_params_dirty = 1;
    if (Regadd == 4U) {
        UpdateAdmittanceRestartFlag(val);
        UpdateExperimentEvent(val);
    }

    modbus.sendbuf[send_len++] = modbus.myadd;
    modbus.sendbuf[send_len++] = 0x06;
    modbus.sendbuf[send_len++] = Regadd >> 8;
    modbus.sendbuf[send_len++] = Regadd & 0xFF;
    modbus.sendbuf[send_len++] = val >> 8;
    modbus.sendbuf[send_len++] = val & 0xFF;

    crc = Modbus_CRC16(modbus.sendbuf, send_len);
    modbus.sendbuf[send_len++] = crc >> 8;
    modbus.sendbuf[send_len++] = crc & 0xFF;

    if (HAL_UART_Transmit_IT(&huart1, modbus.sendbuf, send_len) == HAL_OK) {
        if (uartTxSemHandle != NULL) {
            (void)osSemaphoreAcquire(uartTxSemHandle, 100);
        }
    }
}

/**
 * @brief 16 功能码处理：写多个寄存器
 */
void Modbus_Func16(const uint8_t *buffer, uint16_t length)
{
    uint16_t Regadd, Reglen, crc;
    uint16_t i, send_len = 0;

    if (buffer == NULL || length < 9U)
    {
        return;
    }

    Regadd = (uint16_t)(buffer[2] * 256U + buffer[3]);
    Reglen = (uint16_t)(buffer[4] * 256U + buffer[5]);

    if (Regadd + Reglen > 100) return;

    for (i = 0; i < Reglen; i++)
    {
        Reg[Regadd + i] = (uint16_t)(buffer[7 + i * 2] * 256U + buffer[8 + i * 2]);
    }
    if (Regadd <= 4U && (Regadd + Reglen) > 4U) {
        UpdateAdmittanceRestartFlag(Reg[4]);
        UpdateExperimentEvent(Reg[4]);
    }
    // 如果本次写入的寄存器区间与 PID 参数区间 [5,13] 有重叠, 打上脏标记
    if (Regadd <= PID_REG_KD_SPD && (Regadd + Reglen) > PID_REG_KP_POS)
        pid_params_dirty = 1;

    modbus.sendbuf[send_len++] = modbus.myadd;
    modbus.sendbuf[send_len++] = 0x10;
    modbus.sendbuf[send_len++] = Regadd >> 8;
    modbus.sendbuf[send_len++] = Regadd & 0xFF;
    modbus.sendbuf[send_len++] = Reglen >> 8;
    modbus.sendbuf[send_len++] = Reglen & 0xFF;

    crc = Modbus_CRC16(modbus.sendbuf, send_len);
    modbus.sendbuf[send_len++] = crc >> 8;
    modbus.sendbuf[send_len++] = crc & 0xFF;

    if (HAL_UART_Transmit_IT(&huart1, modbus.sendbuf, send_len) == HAL_OK) {
        if (uartTxSemHandle != NULL) {
            (void)osSemaphoreAcquire(uartTxSemHandle, 100);
        }
    }
}
