#ifndef __RS485_H
#define __RS485_H

#include "main.h"
#include "cmsis_os.h"
#include "rs485_crc.h"

// PB12 连接 DE/RE，控制 RS485 通信流向,使用了板载RS485，不再使用使能引脚，改为直接在发送函数里切换模式
//#define RS485_TX_ENABLE  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)   // 1，发送
//#define RS485_RX_ENABLE  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET) // 0，接收

#define MODBUS_BUF_SIZE 200
#define MODBUS_RX_QUEUE_LENGTH 4

#define RS485_SLAVE_ADDR 0x02 // 从机地址（可根据需要修改）

// LED 闪烁参数: 4 次翻转 = 2 闪, 总时长 = BLINK_PERIOD × 4
#define LED_BLINK_PERIOD_MS  50U

// PID 参数 Modbus 寄存器索引 (Reg[5..13])
#define PID_REG_KP_POS  5
#define PID_REG_KI_POS  6
#define PID_REG_KD_POS  7
#define PID_REG_KP_ANG  8
#define PID_REG_KI_ANG  9
#define PID_REG_KD_ANG  10
#define PID_REG_KP_SPD  11
#define PID_REG_KI_SPD  12
#define PID_REG_KD_SPD  13

// 导纳控制参数寄存器 (case 6)
#define ADM_REG_K  15    // 接触刚度 ×1000
#define ADM_REG_B  16    // 回弹刚度 ×1000
#define ADM_REG_DAMP 17  // 虚拟阻尼 ×1000 (16=0.016, 越小响应越快)

// SPI采集位按实际电机编号定义：bit1/2/3分别为中指/无名指/小指。
// bit0及其他位不绑定传感器，采集开关不控制电机运动使能。
#define TACTILE_REG_ENABLE 18U
#define TACTILE_DEFAULT_ENABLE 0x000EU

// CAN统一开关只允许0/1；频率单位Hz，两条反馈共用，复位恢复默认。
#define CAN_REPORT_REG_ENABLE 19U
#define CAN_REPORT_REG_FREQUENCY 20U
#define CAN_REPORT_DEFAULT_ENABLE 1U
#define CAN_REPORT_DEFAULT_FREQUENCY 20U
#define CAN_REPORT_MIN_FREQUENCY 1U
#define CAN_REPORT_MAX_FREQUENCY 100U

// Modbus_GetCanReportConfig：原子复制两个上报寄存器并防御非法内存值
// 参数：enabled - 开关输出；frequency - Hz输出，地址不可为空
// 返回值：无；正常值直接反映寄存器内容
void Modbus_GetCanReportConfig(uint16_t *enabled, uint16_t *frequency);

typedef struct
{
    uint16_t length;
    uint8_t  data[MODBUS_BUF_SIZE];
} MODBUS_FrameTypeDef;

// MODBUS 协议-参数结构体
typedef struct
{
    // 作为从机时使用
    uint8_t  myadd;             // 本设备从机地址
    uint8_t  rcbuf[MODBUS_BUF_SIZE];        // modbus 接收缓冲区
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
extern volatile uint8_t pid_params_dirty; /* PID 寄存器 (Reg[5..13]) 被写时置 1 */
extern osTimerId_t ledBlinkTimerHandle;    /* RS485 收发触发的 LED 闪烁定时器 */
extern osMessageQueueId_t modbusRxQueueHandle;
extern osSemaphoreId_t uartTxSemHandle; // UART 发送完成信号量


void Modbus_Send_Byte(uint8_t ch);
void Modbus_Init(void);
void Modbus_Event(const MODBUS_FrameTypeDef *frame);
void LedBlinkTimerCallback(void *argument);  /* RS485 LED 双闪回调 */
void Modbus_Func3(const uint8_t *buffer, uint16_t length);
void Modbus_Func6(const uint8_t *buffer, uint16_t length);
void Modbus_Func16(const uint8_t *buffer, uint16_t length);

// 新增: 串口空闲中断回调处理
void RS485_UART_IDLE_CallBack(void);

#endif
