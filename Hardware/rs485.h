#ifndef __RS485_H
#define	__RS485_H

#include "main.h"
#include "rs485_crc.h"
//PB12连接DE/RE，控制RS485通信流向
//#define RS485_TX_ENABLE  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)//1，发送
//#define RS485_RX_ENABLE  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)//0，接收

//MODBUS协议-各参数结构体 
typedef struct 
{
	//作为从机时使用
 	uint8_t  myadd;        //本设备从机地址
	uint8_t  rcbuf[100];   //modbus接受缓冲区
	uint8_t  timout;       //modbus数据持续时间
	uint8_t  recount;      //modbus端口接收到的数据个数
	uint8_t  timrun;       //modbus定时器是否计时标志
	uint8_t  reflag;       //modbus一帧数据接受完成标志位
	uint8_t  sendbuf[100]; //modbus接发送缓冲区
	
	//作为主机添加部分
	uint8_t Host_Txbuf[20];	//modbus发送数组	
	uint8_t slave_add;		//要匹配的从机设备地址（做主机实验时使用）
	uint8_t Host_send_flag;//主机设备发送数据完毕标志位
	int32_t Host_Sendtime;//发送完一帧数据后时间计数
	uint8_t Host_time_flag;//发送时间到标志位，=1表示到发送数据时间了
	uint8_t Host_End;//接收数据后处理完毕
	
}MODBUS;

//typedef enum {
//    MODBUS_CMD_NONE = 0,
//    MODBUS_CMD_NEW  = 1,   /* 中断里置位：有新帧到 */
//    MODBUS_CMD_BUSY = 2    /* 主循环里置位：正在执行本帧 */
//} modbus_cmd_state_t;

//volatile modbus_cmd_state_t modbus2_cmd_state = MODBUS_CMD_NONE;  // 命令状态机

extern MODBUS modbus1;//定义MODBUS结构体类型的变量modbus-对应串口1的1号485，与手指485相连
extern MODBUS modbus2;//定义MODBUS结构体类型的变量modbus2-对应串口2的2号485，接收上位机消息
extern uint16_t Reg[100];//从机存储数据的寄存器

//做从机的函数
void modbus2_Send_Byte(uint8_t ch);//485串口2发一个字节数据的函数
void modbus2_Init(void);//modbus2初始化，定义从机地址
void modbus2_Event(void);//modbus2事件处理函数，做CRC校验+根据功能码进入不同的处理函数
void modbus2_Func3(void);//0x03功能码处理函数，主机读取从机寄存器中的数据
void modbus2_Func6(void);//0x06功能码处理函数，主机向从机的1个寄存器写入值
void modbus2_Func16(void);//0x16功能码处理函数，主机向从机的多个寄存器写入数据
//做主机的函数
void modbus1_Send_Byte(uint8_t ch);//485串口1发一个字节数据的函数
void Host_write06_slave(uint8_t slave,uint8_t fun,uint16_t StartAddr,uint16_t num);//0x06：主机向指定从机的一个寄存器写入数据
void Host_Func6(void);//0x06：主机写入后，接收被写入从机返回的数据
void Host_write16_slave(uint8_t slave,uint8_t fun,uint16_t StartAddr,uint16_t REnum,uint8_t BYTEnum,uint16_t* ModbusData);//0x10：主机向从机的多个寄存器中写入指定数据
void Host_Func16(void);//0x10：主机写入后，接收被写入从机返回的数据


//uint16_t Modbus_CRC16(uint8_t *pData, uint16_t len);//CRC16校验计算函数

#endif
