/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor.h"
#include "rs485.h"
#include "adc.h"
#include "stdio.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t ADC_NativeValue[80] = {0}; // 原始 AD 采样值
uint16_t ADC_SumValue[4] = {0};     // 累加器

osMutexId_t xMotorDataMutexHandle;
const osMutexAttr_t xMotorDataMutex_attributes = {
  .name = "xMotorDataMutex"
};

osThreadId_t motorControlTaskHandle;
const osThreadAttr_t motorControlTask_attributes = {
  .name = "motorControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t modbusCommTaskHandle;
const osThreadAttr_t modbusCommTask_attributes = {
  .name = "modbusCommTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t sensorProcessTaskHandle;
const osThreadAttr_t sensorProcessTask_attributes = {
  .name = "sensorProcessTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t systemMonitorTaskHandle;
const osThreadAttr_t systemMonitorTask_attributes = {
  .name = "systemMonitorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vMotorControlTask(void *argument);
void vModbusCommTask(void *argument);
void vSensorProcessTask(void *argument);
void vSystemMonitorTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  xMotorDataMutexHandle = osMutexNew(&xMotorDataMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  motorControlTaskHandle  = osThreadNew(vMotorControlTask, NULL, &motorControlTask_attributes);
  modbusCommTaskHandle    = osThreadNew(vModbusCommTask, NULL, &modbusCommTask_attributes);
  sensorProcessTaskHandle = osThreadNew(vSensorProcessTask, NULL, &sensorProcessTask_attributes);
  systemMonitorTaskHandle = osThreadNew(vSystemMonitorTask, NULL, &systemMonitorTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  vTaskDelete(NULL); // 删除默认任务，没这个默认任务cubeMX报错
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief 电机控制任务 (最高实时优先级)
  * @param argument: 未使用
  * @retval None
  * @note 绝对的 10ms 周期执行闭环 PID 计算
  */
void vMotorControlTask(void *argument)
{
  uint32_t PreviousWakeTime = osKernelGetTickCount();
 // uint32_t i = 0,j = 0; 
  for(;;)
  {
    // 绝对延时 10ms 周期，确保控制环路稳定
    osDelayUntil(PreviousWakeTime + 10); 
    PreviousWakeTime = osKernelGetTickCount();

    // 获取互斥锁，保护串口指令与本地闭环计算的数据安全
    if (xMotorDataMutexHandle != NULL) {
     // i = osKernelGetTickCount();
      osMutexAcquire(xMotorDataMutexHandle, osWaitForever);
      Motor_Control_Loop(); // 执行电机位置/速度内环闭环计算并输出PWM
      osMutexRelease(xMotorDataMutexHandle);
     /* i = osKernelGetTickCount() - i;
      j++;
      if(j > 50) {
        printf("Motor Control Loop Execution Time: %lu ms\r\n", i);
        j = 0;
      }在这里测试了一下一次电机控制循环要多长时间，实测i==0，用时<1ms,并且在一个时间片里面任务可以完成*/
    }
  }
}

/**
  * @brief Modbus 通信解析任务 (高优先级)
  * @param argument: 未使用
  * @retval None
  * @note 周期性检查空闲中断标记并解析Modbus数据
  */
void vModbusCommTask(void *argument)
{
  for(;;)
  {
    // 降低检查频率，并先检查标志位再申请锁，极大地减小对控制任务的干扰
    osDelay(10);
    
    if (modbus.reflag != 0) { // 仅在 DMA+IDLE 收到一帧完整数据后才处理
      if (xMotorDataMutexHandle != NULL) {
        if (osMutexAcquire(xMotorDataMutexHandle, 10) == osOK) { // 尝试拿锁，超时 10ms
          Modbus_Event(); 
          osMutexRelease(xMotorDataMutexHandle);
        }
      }
    }
  }
}

/**
  * @brief 传感器处理任务 (普通优先级)
  * @param argument: 未使用
  * @retval None
  * @note 处理 ADC 多通道数据的滤波
  */
void vSensorProcessTask(void *argument)
{
  uint8_t i, j;
  for(;;)
  {
    // 20ms 周期执行滤波处理，不需要极高的实时性
    osDelay(20);
    // 关节 AD 值采集与平均滤波（每个通道 20次 采样取平均）
    for(j = 0; j < 4; j++) {
      for(i = 0; i < 20; i++) {
        ADC_SumValue[j] += ADC_NativeValue[4*i + j];
      }
      ADC_HallValue[j] = ADC_SumValue[j] / 20;
      ADC_SumValue[j] = 0;
    }
  }
}

/**
  * @brief 系统监视及低频定时任务 (低优先级)
  * @param argument: 未使用
  * @retval None
  * @note 用于处理 200ms 其他逻辑
  */
void vSystemMonitorTask(void *argument)
{
  uint32_t PreviousWakeTime = osKernelGetTickCount();
  int i = 3; // 监视电机 i-1 的状态
  for(;;)
  {
    // 20ms 的 Modbus 定时节拍器
    osDelayUntil(PreviousWakeTime + 20); 
    PreviousWakeTime = osKernelGetTickCount();
    /*打印电机1的目标速度，实际速度，目标位置，实际位置*/
   // printf("Motor 1 Target Speed: %d, Actual Speed: %d\r\n", (int)motor[i].TargetSpeed, (int)motor[i].CurrentSpeed);
   // printf("Motor 1 Target Angle: %d, Actual Angle: %d\r\n", (int)motor[i].TargetAngle, (int)motor[i].CurrentAngle);
    printf("%d,%d,%d,%d\r\n", 0xff,(int)motor[i].TargetAngle, (int)motor[i].CurrentAngle,0xfe);// 以特定格式打印电机状态，方便上位机解析

    Modbus_Timer_Loop(); // 更新 Modbus 超时接收和 1s 定时发送计时器，其实超时接收已经被IDLE中断回调处理了
  }
}
/* USER CODE END Application */



