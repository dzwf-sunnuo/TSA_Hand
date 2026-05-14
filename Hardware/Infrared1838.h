//#include "main.h"
#ifndef __InfraredRemote_H__
#define __InfraredRemote_H__
////野火遥控器【白】-键值
#define IR_1 	  0x30
#define IR_2 	  0x18
#define IR_3 	  0x7a
#define IR_4 	  0x10
#define IR_5 	  0x38
#define IR_6 	  0x5a
#define IR_7 	  0x42
#define IR_8 	  0x4a
#define IR_9 	  0x52
#define IR_0 	  0x68
#define IR_UP 	  0x02
#define IR_Left   0xe0
#define IR_Right  0x90
#define IR_Down   0x98
#define IR_Pause  0xa8
#define IR_Power  0xa2
#define IR_Xing   0xe2
#define IR_Test   0x22
#define IR_Return 0xc2
#define IR_Clear  0xb0

//弯曲关节，对应白色遥控器键值
#define IR_bend1  0x68
#define IR_bend2  0xb0
#define IR_bend3  0x30
#define IR_bend4  0x7a
#define IR_bend5  0x38
#define IR_bend6  0x42
#define IR_bend7  0x52


//光子遥控器【黑】-键值
//#define IR_1 	  0xa2
//#define IR_2 	  0x62
//#define IR_3 	  0xe2
//#define IR_4 	  0x22
//#define IR_5 	  0x02
//#define IR_6 	  0xc2
//#define IR_7 	  0xe0//伸直-M1
//#define IR_8 	  0xa8//伸直-M2
//#define IR_9 	  0x90//伸直-M3
//#define IR_Xing   0x68
//#define IR_0 	  0x98
//#define IR_Jing   0xb0
//#define IR_UP 	  0x18//伸直-M4
//#define IR_Right  0x5a//伸直-M5
//#define IR_Down   0x4a//伸直-M6
//#define IR_Left   0x10//伸直-M7
//#define IR_OK 	  0x38
#define IR_Empty  0xFF//空指令

extern uint32_t upcount;//因为TIM1更新中断回调函数需要与TIM6的一起写到time.c内，故给到全局变量
extern volatile uint8_t rcvFlag;//接收标志位需要在main.c内使用

void InfraredRemote_Init(void);
void bitbuffer_to_num(uint8_t num[]);
uint16_t InfraredRemote_Get_KeyNum(void);
uint16_t InfraredRemote_Get_KeyNum2(void);
void InfraredRemote_DumpFrame(UART_HandleTypeDef *huart);


#endif


