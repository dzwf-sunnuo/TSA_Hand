#ifndef __RS485_H
#define __RS485_H

#include "main.h"
#include "rs485_crc.h"

// PB12 连接 DE/RE，控制 RS485 通信流向,使用了板载RS485，不再使用使能引脚，改为直接在发送函数里切换模式
//#define RS485_TX_ENABLE  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)   // 1，发送
//#define RS485_RX_ENABLE  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET) // 0，接收

#define MODBUS_BUF_SIZE 100

#define RS485_SLAVE_ADDR 0x01 // 从机地址（可根据需要修改）

// MODBUS 协议-参数结构体
typedef struct
{
    // 作为从机时使用
    uint8_t  myadd;             // 本设备从机地址
    uint8_t  rcbuf[MODBUS_BUF_SIZE];        // modbus 接收缓冲区
    uint8_t  recount;           // modbus 端口接收到的数据个数
    uint8_t  reflag;            // modbus 一帧数据接收完成标志位
    uint8_t  sendbuf[MODBUS_BUF_SIZE];      // modbus 发送缓冲区

    // 状态与计时 (已移除 8ms 软件断帧变量)
    
    // 作为主机预留
    uint8_t  slave_add;         // 要匹配的从机设备地址
    uint8_t  Host_Txbuf[8];     // 主机发送数组
    uint8_t  Host_send_flag;    // 主机发送完成标志
    int32_t  Host_Sendtime;     // 主机发送计时
    uint8_t  Host_time_flag;    // 主机时间到达标志
    uint8_t  Host_End;          // 主机处理结束
} MODBUS;

extern MODBUS modbus;
extern uint16_t Reg[100];


void Modbus_Send_Byte(uint8_t ch);
void Modbus_Init(void);
void Modbus_Event(void);
void Modbus_Func3(void);
void Modbus_Func6(void);
void Modbus_Func16(void);

// 新增: 串口空闲中断回调处理
void RS485_UART_IDLE_CallBack(void);

#endif
