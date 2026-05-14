/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "rs485.h"
#include "wrist.h"
#include "Infrared1838.h"
#include "String.h"
#include "motion.h"
#include "MyCan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define   CONSTAND_SPEED            0x01
#define   VARIABLE_SPEED            0x02
#define   ID_HAND                   0x020
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t key_num = 0xFF;//红外遥控器发来的动作标号
volatile uint8_t motion_count = 3;//动作次数
volatile uint8_t can_motion_flag = 0;//连续动作是否是can发送的
volatile uint8_t can_num = 5;//单个动作
volatile uint8_t infrared_enable = 1;//使能

extern volatile uint8_t wrist_enabled;  
extern volatile uint8_t wrist_enabled_constand;
extern  uint8_t NUMGesture;
extern  uint8_t NUMAction;//代表当前做的是第几个连续动作
extern  uint8_t NUMActionCycle;//代表连续动作循环的次数
extern  uint8_t NVICFlag;//定时中断发生的标志位
volatile uint8_t up_count,down_count,right_count,left_count;
	uint8_t led = 0;
	volatile uint8_t mode = VARIABLE_SPEED;
extern volatile uint16_t cycle_time;              // 一个来回的时间(ms)，可调
extern volatile uint16_t constant_speed;        // 恒定速度值(0-4000)，对应0-32mm/s
extern volatile uint16_t cycle_time_R;              // 一个来回的时间(ms)，可调,旋转
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MyCan_RxHandler(CAN_Message_t *msg);  // 声明回调函数
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Infrared_Signal_Processing_Function(volatile uint16_t *key_num,uint8_t F)
{
		if(F != 1 || *key_num == 0x00FF)return;
		if(*key_num == IR_8){//8号动作复位  手腕复位
			*key_num  = IR_Empty;//转移到无效的动作指令
				vel_move(0x04,0x00,0x07,0xC6);				
				vel_move1(0x04,0x00,0x07,0xC6);				
				Reg[14] = 0x0001;
				wrist_enabled = 0x00;
				wrist_enabled_constand = 0x00;
				cycle_time = 3000;
				cycle_time_R = 3000;
				uint16_t stop[5] = {0x0064,0x0064,0x1064,0x0064,0x0101};
				Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, stop);
				HAL_Delay(100);
				Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, stop);
		}
		else if(*key_num==IR_1){//1号连续动作变速度摆手
			
//			Reg[14] = 0x0101;
			*key_num  = IR_Empty;//转移到无效的动作指令
			wrist_enabled = 0x01;
			wrist_enabled_constand = 0x00;
		}
		else if(*key_num==IR_2 ){
			wrist_enabled = 0x00;
			wrist_enabled_constand = 0x01;
			*key_num  = IR_Empty;//转移到无效的动作指令
			
		}
		else if(*key_num==IR_3 && NUMActionCycle<motion_count){//1号连续动作-数数字，动作循环1次	
			
		finger_motion(COUNT_NUMBERS,1300);
		NUMActionCycle=NUMActionCycle+1;		  
		}
		
		else if(*key_num==IR_4 && NUMActionCycle<motion_count){//2号连续动作-连续弯折，动作循环3次
		finger_motion(CONTINUOUS_BENDING,300);
		NUMActionCycle=NUMActionCycle+1;		  
		}
		else if(*key_num==IR_5 && NUMActionCycle<motion_count){//
		finger_motion(OTHER_MOTION,1000);
		NUMActionCycle=NUMActionCycle+1;		  
	}
		else if(*key_num==IR_7 && NUMActionCycle<motion_count){//
		finger_motion(CONTINUOUS_FISTING,1500);
		NUMActionCycle=NUMActionCycle+1;		  
	}
	  else if(*key_num==IR_UP){
		   *key_num  = IR_Empty;//转移到无效的动作指令，保证只执行一次本内容
			modbus1_Send_Byte('w');
	  }
	  else if(*key_num==IR_Down){
		   *key_num  = IR_Empty;//转移到无效的动作指令，保证只执行一次本内容
	  }
	  else if(*key_num==IR_Xing){//单个动作
			finger_single_action(can_num);
//			printf("%d",key_num);
		  *key_num  = IR_Empty;//转移到无效的动作指令，保证只执行一次本内容
//			printf("%d\n",key_num);

	  }
	  else if(*key_num==IR_UP){//
		   *key_num  = IR_Empty;//转移到无效的动作指令，保证只执行一次本内容
	  }
	  else if(*key_num==IR_Right){//
		   *key_num  = IR_Empty;//转移到无效的动作指令，保证只执行一次本内容
	  }
	  else if(*key_num==IR_Down){//
		   *key_num  = IR_Empty;//转移到无效的动作指令，保证只执行一次本内容
	  }
	  else if(*key_num==IR_Left){//
		   *key_num  = IR_Empty;//转移到无效的动作指令，保证只执行一次本内容
	  }
			printf("333%d\n",*key_num);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  modbus2_Init();
	HAL_CAN_MspInit(&hcan1);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&RES2, 1);
  init_uart_receive(&huart3); 
  init_uart_receive1(&huart6); 
	
	
	
// 初始化MyCan模块
  MyCan_Init(&hcan1, MyCan_RxHandler);
//	MyCan_ConfigFilter_Mask32Bit(3, 0x000, 0x000, CAN_RX_FIFO0);// 配置过滤器（根据需要选择）
	MyCan_ConfigFilter_List16Bit(0 , ID_HAND,ID_HAND,ID_HAND,ID_HAND,CAN_RX_FIFO0);
	MyCan_Start();// 启动CAN（接收中断）
		
		
		
////开启红外遥控部分2个中断
  HAL_TIM_Base_Start_IT(&htim3);               //定时器更新时，产出中断
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);   //开启输入捕获通道中断
////  uint8_t i = 0;
	up_count = 5;
	modbus2_Send_Byte(up_count);
//	printf("123");
	uint8_t data[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
	MyCan_SendStdData(0x023, data, 8);		
//	set_wrist_motion_Can(0x10,0x03);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		modbus2_Event();
		Modbus_Slave_Dispatcher();
		InfraredRemote_DumpFrame(&huart2);//打印接收数据帧，调试用
	  HAL_Delay(50);
		if(rcvFlag == 1){
			if(can_motion_flag == 0){
		  key_num = InfraredRemote_Get_KeyNum();}//接收红外遥控器给出的动作标号
			else{can_motion_flag = 0;}
			can_motion_flag = 0;
		  rcvFlag = 0;//清零红外接收标志位，等待下一次红外遥控器发送数据
		  NUMActionCycle = 0;//【连续动作循环次数NUMActionCycle置为0，表明这是第一次循环】【非常重要】
		  NUMGesture = 0;//【手势序号NUMGesture置为0，保证连续指令总会从手势0开始】【非常重要】
//			infrared_enable = 1;
		}
		Infrared_Signal_Processing_Function(&key_num,infrared_enable);
//		printf("444%d\n",key_num);
//*******************************************************************************************************
//				HAL_Delay(1000);
//				modbus2_Send_Byte(rcvFlag);
//        /* 每隔1秒发送一个标准数据帧，ID为0x123，数据为0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07 */
//        uint8_t data[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
//        MyCan_SendStdData(0x023, data, 8);		
//*******************************************************************************************************
	}//while
			  	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == htim7.Instance){
		if(modbus2.timrun != 0){
			modbus2.timout++;
			if(modbus2.timout >=8){
				modbus2.timrun = 0;
				modbus2.reflag = 1;
			}
		}
 		modbus2.Host_Sendtime++;
 		if(modbus2.Host_Sendtime>1000){
			
			modbus2.Host_time_flag=1;
 		}
	}	
	
	else if(htim->Instance == htim2.Instance && wrist_enabled)
    {
        updateSmartMotionControl(MODE_PENDULUM,cycle_time);
    }
	else if(htim->Instance == htim2.Instance && wrist_enabled_constand)
    {
        updateSmartMotionControl(MODE_ROTATION,cycle_time_R);
    }
		
		
	else if(htim->Instance == htim3.Instance){
		
			
	}
		
		
}

/**
  * @brief CAN接收回调函数
  * @param msg: 接收到的CAN消息
  * @retval None
  */
void MyCan_RxHandler(CAN_Message_t *msg)
{
//				modbus2_Send_Byte(msg ->data[0]);
//    /* 在这里处理接收到的CAN消息 */
//    printf("Received CAN message: ID=0x%X, Length=%d, Data: ", msg->id, msg->length);
//    for(int i = 0; i < msg->length; i++)
//    {
//        printf("%02X ", msg->data[i]);
//    }
//    printf("\n");
	// 根据 ID 解析数据
//		printf("121");
	//手腕运动
	if(msg->id == ID_HAND && (msg->data[0] == 0x10 || msg->data[0] == 0x11)){
				set_wrist_motion_Can(msg->data[0],msg->data[2]);}
	
	//手指连续运动
	else if(msg->id == ID_HAND && msg->data[0] == 0x12 ){
			rcvFlag = 1;  //模拟红外信号
			can_motion_flag = 1;
	    switch (msg->data[2])
    {
		case 0x00: key_num = IR_8; motion_count = 0		    	; break; //停止
		case 0x01: key_num = IR_4; motion_count = msg->data[3]; break; //连续弯折手指
		case 0x02: key_num = IR_3; motion_count = msg->data[3]; break; //数数字
		case 0x03: key_num = IR_7; motion_count = msg->data[3]; break; //握拳
		}
	}
	//手指单个动作
	else if(msg->id == ID_HAND && msg->data[0] == 0x13 ){
		rcvFlag = 1;  //模拟红外信号
	  can_motion_flag = 1;
		key_num = IR_Xing;
	  can_num = (msg->data[2]);
	}
	//全部停止
	else if(msg->id == ID_HAND && msg->data[0] == 0x14 ){
//		set_wrist_motion_Can(0x10,0x00);
//		finger_single_action(0x00);		
		rcvFlag = 1;		
	  can_motion_flag = 1;
		key_num = IR_8;
		
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
