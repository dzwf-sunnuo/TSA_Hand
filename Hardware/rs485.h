#ifndef __RS485_H
#define __RS485_H

#include "main.h"
#include "rs485_crc.h"
#include <stdbool.h>

// MODBUS：保存上位机从机端或手指总线主机端的协议状态
// 参数：接收缓冲区容量固定，接收长度必须在解析前校验
// 返回值：无
typedef struct 
{
    uint8_t  myadd;           /* 从机设备地址 */
    uint8_t  rcbuf[100];      /* 接收缓冲区 */
    uint8_t  timout;          /* 帧超时计数 */
    volatile uint8_t recount; /* 中断交付的接收字节计数 */
    uint8_t  timrun;          /* 超时定时器运行标志 */
    volatile uint8_t reflag;  /* 中断交付的一帧数据接收完成标志 */
    uint8_t  sendbuf[100];    /* 发送缓冲区 */
    
    uint8_t  Host_Txbuf[64];  /* 作为主机时的发送缓冲区 */
    uint8_t  slave_add;       /* 目标从机地址 */
    uint8_t  Host_send_flag;  /* 发送完成标志 */
    int32_t  Host_Sendtime;   /* 发送后的时间计数 */
    uint8_t  Host_time_flag;  /* 定时发送标志 */
    uint8_t  Host_End;        /* 接收处理完成标志 */
} MODBUS;

/* 全局变量声明 */
extern MODBUS modbus1;      /* 历史名称保留：USART2连接手指驱动板，主机模式 */
extern MODBUS modbus2;      /* 历史名称保留：USART1连接PC上位机，从机模式 */
extern uint16_t Reg[100];   /* 共享寄存器数组 */

// PID 参数寄存器 (主板 Reg[20..28], 对上位机暴露)
// 存储格式: uint16 = float × 100 (例: 500 → 5.0f)
#define PID_REG_MASTER_BASE  20    // 主板 PID 起始寄存器
#define PID_REG_COUNT         9    // 3环 × 3参数
#define PID_REG_SLAVE_ADDR    32   // 从板PID迁到Reg[32]，与单路目标/模式区分离

// 七个电机交错目标/模式占用Reg[4..17]，分发到一号板8个、二号板6个寄存器。
#define HAND_CONTROL_BASE 4U
#define HAND_CONTROL_COUNT 14U
#define HAND_BOARD1_COUNT 8U
#define HAND_BOARD2_COUNT 6U
#define HAND_BOARD2_BASE (HAND_CONTROL_BASE + HAND_BOARD1_COUNT)
// 原Reg[14..17]迁移到40～43，避免新手指目标误触发推杆/手腕动作。
#define WRIST_REG_CONTROL 40U
#define WRIST_REG_CYCLE 41U
#define WRIST_REG_PRESET1 42U
#define WRIST_REG_PRESET2 43U

// 统一导纳参数追加到既有业务区之后，不移动任何原控制寄存器；数值均为参数乘1000。
#define ADM_REG_MASTER_K_CONTACT 44U
#define ADM_REG_MASTER_K_RETURN 45U
#define ADM_REG_MASTER_DAMP 46U
#define ADM_REG_COUNT 3U
#define ADM_REG_SLAVE_BASE 15U
#define ADM_DEFAULT_K_CONTACT 50U
#define ADM_DEFAULT_K_RETURN 200U
#define ADM_DEFAULT_DAMP 25U

#define MODBUS_REG_COUNT 100U
#define UART4_REPORT_REG_ENABLE 30U
#define UART4_REPORT_REG_FREQUENCY 31U
#define UART4_REPORT_REG_RATIO 32U
#define UART4_REPORT_REG_FORCE_MIN 33U
#define UART4_REPORT_DEFAULT_ENABLE 1U
#define UART4_REPORT_DEFAULT_FREQUENCY 5U
#define UART4_REPORT_DEFAULT_RATIO 10U
#define UART4_REPORT_DEFAULT_FORCE_MIN 30U
#define UART4_REPORT_MAX_FREQUENCY 100U

/* 从机模式函数 (响应PC) */
// modbus2_Init：初始化USART1上位机协议状态及寄存器默认值
// 参数：无
// 返回值：无
void modbus2_Init(void);
// modbus2_Event：处理一帧请求，仅成功控制写入触发相应动作
// 参数：无，使用modbus2接收缓冲区
// 返回值：无
void modbus2_Event(void);
// Modbus_ProcessRequest：通过公共校验与执行入口处理一路上位机请求
// 参数：data - 稳定的请求数据；length - 含CRC长度；reply - true仅在USART1回复，false完全静默
// 返回值：true表示请求合法并完成处理，false表示校验或业务参数失败；只能由ModbusPC任务调用
bool Modbus_ProcessRequest(const uint8_t *data, uint16_t length, bool reply);
// Modbus_IsRequestFrame：检查主板地址、请求长度上限及项目原有CRC线序
// 参数：data - 请求起始位置；length - 含CRC长度
// 返回值：true表示地址和CRC正确；不检查寄存器范围及功能码语义
bool Modbus_IsRequestFrame(const uint8_t *data, uint16_t length);
// modbus2_Send_Byte：通过USART1发送一个上位机协议字节
// 参数：ch - 待发送字节
// 返回值：无
void modbus2_Send_Byte(uint8_t ch);
// modbus2_Func3：处理已校验CRC的读保持寄存器请求
// 参数：无
// 返回值：无
void modbus2_Func3(void);
// modbus2_Func6：校验并提交单寄存器写入
// 参数：无
// 返回值：无
void modbus2_Func6(void);
// modbus2_Func16：整帧校验后原子提交多寄存器写入
// 参数：无
// 返回值：无
void modbus2_Func16(void);

/* 主机模式函数 (控制手指板) */
// ModbusMaster_Init：创建USART2主机发送互斥锁
// 参数：无，必须在内核初始化后、任务启动前调用
// 返回值：非0表示成功
uint8_t ModbusMaster_Init(void);
// modbus1_Send_Byte：通过USART2发送一个手指总线字节
// 参数：ch - 待发送字节
// 返回值：无
void modbus1_Send_Byte(uint8_t ch);
// Host_write06_slave：通过USART2向指定从板写一个寄存器
// 参数：slave - 从板地址；fun - 功能码；StartAddr - 地址；num - 数值
// 返回值：无，结果保存在主机发送状态中
void Host_write06_slave(uint8_t slave, uint8_t fun, uint16_t StartAddr, uint16_t num);
// Host_write16_slave：通过USART2向指定从板批量写寄存器
// 参数：slave - 地址；fun - 功能码；StartAddr - 首地址；REnum - 数量；BYTEnum - 字节数；ModbusData - 数据
// 返回值：无，拒绝超过主机缓冲区容量的请求
void Host_write16_slave(uint8_t slave, uint8_t fun, uint16_t StartAddr, uint16_t REnum, uint8_t BYTEnum, uint16_t* ModbusData);

// Host_write_legacy_action：将内部预设表的四目标加公共模式转换为新交错布局
// 参数：slave - 一号/二号板地址；legacy - 长度5的原动作表项，不接受上位机旧帧
// 返回值：无，二号板丢弃原motor0占位，仅发送原motor1～3
void Host_write_legacy_action(uint8_t slave, const uint16_t legacy[5]);

#endif
