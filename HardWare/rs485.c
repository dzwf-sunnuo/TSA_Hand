#include "rs485.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

MODBUS modbus;
uint16_t Reg[100] = {0};

/**
 * @brief 485 串口发送单字节 (保留原 API)
 * @param ch 发送的字符
 */
void Modbus_Send_Byte(uint8_t ch)
{
    RS485_TX_ENABLE;
    HAL_UART_Transmit(&huart1, &ch, 1, 10);
    // 注意：单字节发送后不立即切换 RX，由上层整包发送完成后统一处理
}

/**
 * @brief Modbus 初始化
 */
void Modbus_Init()
{
    memset(&modbus, 0, sizeof(MODBUS));
    
    // 初始化寄存器默认值
    Reg[0] = 0x0000; // motor1
    Reg[1] = 0x0000; // motor2
    Reg[2] = 0x0000; // motor3
    Reg[3] = 0x0000; // motor4
    Reg[4] = 0x0000; // 模式与状态

    modbus.myadd = 0x02;    // 从机设备地址为 2
    
    RS485_RX_ENABLE;        // 默认进入接收模式
    
    // 开启串口空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // 开启 DMA 接收
    HAL_UART_Receive_DMA(&huart1, modbus.rcbuf, MODBUS_BUF_SIZE);
}

/**
 * @brief 串口空闲中断回调处理函数
 * @note  在 stm32f4xx_it.c 的 USART1_IRQHandler 中调用
 */
void RS485_UART_IDLE_CallBack(void)
{
    uint32_t temp;
    
    // 清除空闲中断标志位
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        
        // 停止当前 DMA 接收
        HAL_UART_DMAStop(&huart1);
        
        // 计算接收到的数据长度
        // CNDTR 寄存器指示剩余待传输量，用总长度减去剩余量即为已接收量
        temp = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        modbus.recount = MODBUS_BUF_SIZE - temp;
        modbus.reflag = 1; // 标记一帧接收完成
        
        // 重新开启 DMA 接收，准备接收下一帧
        HAL_UART_Receive_DMA(&huart1, modbus.rcbuf, MODBUS_BUF_SIZE);
    }
}

/**
 * @brief Modbus 事件处理逻辑
 */
void Modbus_Event()
{
    uint16_t crc, rccrc;
    
    if (modbus.reflag == 0) return; // 无新数据
    
    // 基础长度校验
    if (modbus.recount < 4) 
    {
        modbus.recount = 0;
        modbus.reflag = 0;
        return;
    }

    // CRC 校验
    crc = Modbus_CRC16(modbus.rcbuf, modbus.recount - 2);
    rccrc = modbus.rcbuf[modbus.recount - 2] * 256 + modbus.rcbuf[modbus.recount - 1];
    
    if (crc == rccrc)
    {
        if (modbus.rcbuf[0] == modbus.myadd || modbus.rcbuf[0] == 0x00) // 地址匹配或广播
        {
            switch (modbus.rcbuf[1])
            {
                case 3:  Modbus_Func3();  break;
                case 6:  Modbus_Func6();  break;
                case 16: Modbus_Func16(); break;
                default: break;
            }
        }
    }
    
    modbus.recount = 0;
    modbus.reflag = 0;
}

/**
 * @brief 03 功能码处理：读取寄存器
 */
void Modbus_Func3()
{
    uint16_t Regadd, Reglen, crc;
    uint16_t i, send_len = 0;

    Regadd = modbus.rcbuf[2] * 256 + modbus.rcbuf[3];
    Reglen = modbus.rcbuf[4] * 256 + modbus.rcbuf[5];

    // 边界检查
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

    // 485 发送处理
    RS485_TX_ENABLE;
    HAL_UART_Transmit(&huart1, modbus.sendbuf, send_len, 100);
    // 重要：必须等待发送完成 TC 标志，否则切回 RX 会导致最后一个字节没发完
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
    RS485_RX_ENABLE;
}

/**
 * @brief 06 功能码处理：写单个寄存器
 */
void Modbus_Func6()
{
    uint16_t Regadd, val, crc;
    uint16_t send_len = 0;

    Regadd = modbus.rcbuf[2] * 256 + modbus.rcbuf[3];
    val = modbus.rcbuf[4] * 256 + modbus.rcbuf[5];

    if (Regadd >= 100) return;

    Reg[Regadd] = val;

    // 回应：原样返回请求包（带正确的地址和功能码）
    modbus.sendbuf[send_len++] = modbus.myadd;
    modbus.sendbuf[send_len++] = 0x06;
    modbus.sendbuf[send_len++] = Regadd >> 8;
    modbus.sendbuf[send_len++] = Regadd & 0xFF;
    modbus.sendbuf[send_len++] = val >> 8;
    modbus.sendbuf[send_len++] = val & 0xFF;

    crc = Modbus_CRC16(modbus.sendbuf, send_len);
    modbus.sendbuf[send_len++] = crc >> 8;
    modbus.sendbuf[send_len++] = crc & 0xFF;

    RS485_TX_ENABLE;
    HAL_UART_Transmit(&huart1, modbus.sendbuf, send_len, 100);
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
    RS485_RX_ENABLE;
}

/**
 * @brief 16 功能码处理：写多个寄存器
 */
void Modbus_Func16()
{
    uint16_t Regadd, Reglen, crc;
    uint16_t i, send_len = 0;

    Regadd = modbus.rcbuf[2] * 256 + modbus.rcbuf[3];
    Reglen = modbus.rcbuf[4] * 256 + modbus.rcbuf[5];

    if (Regadd + Reglen > 100) return;

    for (i = 0; i < Reglen; i++)
    {
        Reg[Regadd + i] = modbus.rcbuf[7 + i * 2] * 256 + modbus.rcbuf[8 + i * 2];
    }

    // 回应
    modbus.sendbuf[send_len++] = modbus.myadd;
    modbus.sendbuf[send_len++] = 0x10;
    modbus.sendbuf[send_len++] = Regadd >> 8;
    modbus.sendbuf[send_len++] = Regadd & 0xFF;
    modbus.sendbuf[send_len++] = Reglen >> 8;
    modbus.sendbuf[send_len++] = Reglen & 0xFF;

    crc = Modbus_CRC16(modbus.sendbuf, send_len);
    modbus.sendbuf[send_len++] = crc >> 8;
    modbus.sendbuf[send_len++] = crc & 0xFF;

    RS485_TX_ENABLE;
    HAL_UART_Transmit(&huart1, modbus.sendbuf, send_len, 100);
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
    RS485_RX_ENABLE;
}
