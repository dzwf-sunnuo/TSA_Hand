#ifndef __RS485_H
#define	__RS485_H

#include "main.h"
#include "rs485_crc.h"
//PB12连接DE/RE，控制RS485通信流向
#define RS485_TX_ENABLE  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)//1，发送
#define RS485_RX_ENABLE  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)//0，接收

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
	uint8_t Host_Txbuf[8];	//modbus发送数组	
	uint8_t slave_add;		//要匹配的从机设备地址（做主机实验时使用）
	uint8_t Host_send_flag;//主机设备发送数据完毕标志位
	int32_t Host_Sendtime;//发送完一帧数据后时间计数
	uint8_t Host_time_flag;//发送时间到标志位，=1表示到发送数据时间了
	uint8_t Host_End;//接收数据后处理完毕
}MODBUS;

extern MODBUS modbus;//定义MODBUS结构体类型的变量modbus
extern uint16_t Reg[100];//从机存储数据的寄存器

void Modbus_Send_Byte(uint8_t ch);//485串口发一个字节数据的函数

void Modbus_Init(void);//Modbus初始化，定义从机地址
void Modbus_Event(void);//Modbus事件处理函数，做CRC校验+根据功能码进入不同的处理函数
void Modbus_Func3(void);//0x03功能码处理函数，主机读取从机寄存器中的数据
void Modbus_Func6(void);//0x06功能码处理函数，主机向从机的1个寄存器写入值
void Modbus_Func16(void);//0x16功能码处理函数，主机向从机的多个寄存器写入数据

//uint16_t Modbus_CRC16(uint8_t *pData, uint16_t len);//CRC16校验计算函数

#endif
