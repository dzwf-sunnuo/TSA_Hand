#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "string.h"
#include "Infrared1838.h"
uint32_t upcount;        //记录更新事件产生的次数
uint32_t valueUp;        //捕获到上升沿时刻的值
uint32_t valueDown;      //捕获到下降沿时刻的值
uint8_t  isUpCapture=1;  //判断是否为上升沿捕获，初始需要捕获上升沿，给1
uint16_t PluseWidth;     //脉宽（上升沿捕获和下降沿捕获时间的差值）
uint16_t buffer[128]={0};//接收数据缓冲区
uint16_t bufferID=0;     //数据存放的位置
volatile uint8_t rcvFlag = 0;       //接收成功标志位
static uint16_t frameBuffer[33] = {0}; // 一帧数据快照：同步码1个 + 数据位32个


char printfbuff[128]={0};
uint8_t num[4]={0};


void InfraredRemote_Init(void)
{
	  HAL_TIM_Base_Start_IT(&htim3);               //定时器更新时，产出中断
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);   //开启输入捕获通道中断
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //TIM1更新中断回调函数
//{
//	upcount++;
//}放到主循环去

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)    //输入捕获中断回调函数
{
	if (htim->Instance != TIM3 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_3)
	{
		return;
	}

	if(isUpCapture)  //如果是上升沿捕获
	{
		valueUp=HAL_TIM_ReadCapturedValue(htim ,TIM_CHANNEL_3);   //读取捕获上升沿时刻的计数器值
		isUpCapture=0;
		__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);  //设置为下降沿捕获
		upcount=0;  //清零更新计数器
	}
	else  //如果是下降沿捕获
	{
		valueDown=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
		isUpCapture=1;
		__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_RISING);
		PluseWidth=(uint16_t)(valueDown+upcount*65536u -valueUp);    // upcount*65536 防止计数器溢出   单位是1us
		
		//同步码 (低电平9ms,高电平4.5ms)
		if(PluseWidth>4000 && PluseWidth<5000)
		{
			bufferID=0;
			buffer[bufferID++]=PluseWidth;
		}
		else if (bufferID>0)   //收到了同步码
		{
			if (bufferID < (sizeof(buffer) / sizeof(buffer[0])))
			{
				buffer[bufferID++] = PluseWidth;
			}
			else
			{
				bufferID = 0;
				return;
			}

			if (bufferID >= 33)   // 1同步码头 + 32位数据
			{
				for(uint8_t i=0;i<33;i++)
				{
					frameBuffer[i] = buffer[i];
				}
				rcvFlag=1;
				bufferID=0;    //继续查找同步码
			}
		}
	}
	
}


void bitbuffer_to_num(uint8_t num[])  //将buffer中的数据转换为4字节数字存储在num中
{
	num[0]=0;
	num[1]=0;
	num[2]=0;
	num[3]=0;
	for(int i=0;i<32;i++)       //按8位分割
	{
		if (frameBuffer[i+1] <1000)  //二进制为为0
		{
			num[i/8] = num[i/8]<<1;
		}
		else                    //二进制为为1
		{
			num[i/8] = num[i/8]<<1;
			num[i/8] |= 0x01;
		}
	}
}


uint16_t InfraredRemote_Get_KeyNum(void)
{
	if (rcvFlag==1)
	  {
		  bitbuffer_to_num(num);
//		  HAL_UART_Transmit(&huart1,(uint8_t *)"\r\n",2,HAL_MAX_DELAY);
		  
//		  rcvFlag=0;
	  }	  
	  return (uint16_t)num[2];
}

uint16_t InfraredRemote_Get_KeyNum2(void)
{
	if (rcvFlag==1)
	  {
		  for(int i=0;i<33;i++)
		  {

			  sprintf(printfbuff,"%u ",buffer[i]);
			  HAL_UART_Transmit(&huart4,(uint8_t *)printfbuff,strlen(printfbuff),HAL_MAX_DELAY);
		  }
		  HAL_UART_Transmit(&huart4,(uint8_t *)"\r\n",2,HAL_MAX_DELAY);
		  
		  rcvFlag=0;
	  }
	  
	  return (uint16_t)num[2];
}

/*专门写了一个用于打印接收数据帧的函数，不在中断里面打印*/
void InfraredRemote_DumpFrame(UART_HandleTypeDef *huart)
{
	if (huart == NULL)
	{
		return;
	}

	if (rcvFlag==1)
	{
		for(int i=0;i<33;i++)
		{
			sprintf(printfbuff,"%u ",frameBuffer[i]);
			HAL_UART_Transmit(huart,(uint8_t *)printfbuff,strlen(printfbuff),HAL_MAX_DELAY);
		}
		HAL_UART_Transmit(huart,(uint8_t *)"\r\n",2,HAL_MAX_DELAY);
	}
}




