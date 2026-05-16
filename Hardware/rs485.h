#ifndef __RS485_H
#define __RS485_H

#include "main.h"
#include "rs485_crc.h"

/* Modbus协议参数结构体 */
typedef struct 
{
    uint8_t  myadd;           /* 从机设备地址 */
    uint8_t  rcbuf[100];      /* 接收缓冲区 */
    uint8_t  timout;          /* 帧超时计数 */
    uint8_t  recount;         /* 接收字节计数 */
    uint8_t  timrun;          /* 超时定时器运行标志 */
    uint8_t  reflag;          /* 一帧数据接收完成标志 */
    uint8_t  sendbuf[100];    /* 发送缓冲区 */
    
    uint8_t  Host_Txbuf[20];  /* 作为主机时的发送缓冲区 */
    uint8_t  slave_add;       /* 目标从机地址 */
    uint8_t  Host_send_flag;  /* 发送完成标志 */
    int32_t  Host_Sendtime;   /* 发送后的时间计数 */
    uint8_t  Host_time_flag;  /* 定时发送标志 */
    uint8_t  Host_End;        /* 接收处理完成标志 */
} MODBUS;

/* 全局变量声明 */
extern MODBUS modbus1;      /* 串口1：连接手指驱动板（主机模式） */
extern MODBUS modbus2;      /* 串口2：连接PC上位机（从机模式） */
extern uint16_t Reg[100];   /* 共享寄存器数组 */

/* 从机模式函数 (响应PC) */
void modbus2_Init(void);
void modbus2_Event(void);
void modbus2_Send_Byte(uint8_t ch);
void modbus2_Func3(void);
void modbus2_Func6(void);
void modbus2_Func16(void);

/* 主机模式函数 (控制手指板) */
void modbus1_Send_Byte(uint8_t ch);
void Host_write06_slave(uint8_t slave, uint8_t fun, uint16_t StartAddr, uint16_t num);
void Host_write16_slave(uint8_t slave, uint8_t fun, uint16_t StartAddr, uint16_t REnum, uint8_t BYTEnum, uint16_t* ModbusData);

#endif
