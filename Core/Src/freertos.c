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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor.h"
#include "rs485.h"
#include "adc.h"
#include "power_loss.h"
#include "stdio.h"
#include <string.h>
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
uint16_t ADC_NativeValue[60] = {0}; // DMA 目标缓冲区 (4ch × 15样本)
__CCM_RAM_DATA uint16_t ADC_Snapshot[60] = {0};    // 完成回调中拷贝的快照

osMutexId_t xMotorDataMutexHandle;
const osMutexAttr_t xMotorDataMutex_attributes = {
  .name = "xMotorDataMutex"
};
osSemaphoreId_t Sem_ADC_Done;  // DMA 单次采集完成信号
osMessageQueueId_t modbusRxQueueHandle;
// UART 发送完成信号量（用于替换 busy-wait）
osSemaphoreId_t uartTxSemHandle;

// 1s 周期 Modbus 定时器（替代原先的累加计时方式）
osTimerId_t modbus1sTimerHandle;
const osTimerAttr_t modbus1sTimer_attributes = {
  .name = "modbus1sTimer"
};

// LED 闪烁定时器 (RS485 收发触发双闪, 50ms 周期, 一次性)
osTimerId_t ledBlinkTimerHandle;
const osTimerAttr_t ledBlinkTimer_attributes = {
  .name = "ledBlinkTimer"
};

// 注：暂时停用掉电延迟初始化定时器
// osTimerId_t powerLossInitTimerHandle;
// const osTimerAttr_t powerLossInitTimer_attributes = {
//   .name = "powerLossInitTimer"
// };

osThreadId_t motorControlTaskHandle;
__CCM_RAM_DATA static StaticTask_t motorControlTask_TCB;
__CCM_RAM_DATA static StackType_t motorControlTask_Stack[512];  // 512 words = 2KB
const osThreadAttr_t motorControlTask_attributes = {
  .name = "motorControlTask",
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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t systemMonitorTaskHandle;
const osThreadAttr_t systemMonitorTask_attributes = {
  .name = "systemMonitorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vMotorControlTask(void *argument);
void vModbusCommTask(void *argument);
void vSensorProcessTask(void *argument);
void vSystemMonitorTask(void *argument);
void PowerLossInitTimerCallback(void *argument);
void Modbus1sTimerCallback(void *argument);
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
  Sem_ADC_Done = osSemaphoreNew(1, 0, NULL);  // DMA 单次完成
  uartTxSemHandle = osSemaphoreNew(1, 0, NULL); // UART TX 完成信号量（初始为 0）
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  // 使用 1s 定时器触发 Modbus 定时事件
  modbus1sTimerHandle = osTimerNew(Modbus1sTimerCallback, osTimerPeriodic, NULL, &modbus1sTimer_attributes);
  // RS485 LED 闪烁定时器 (一次性, 由 RS485 收发回调启动)
  ledBlinkTimerHandle  = osTimerNew(LedBlinkTimerCallback, osTimerOnce, NULL, &ledBlinkTimer_attributes);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  modbusRxQueueHandle = osMessageQueueNew(MODBUS_RX_QUEUE_LENGTH, sizeof(MODBUS_FrameTypeDef), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // 静态创建电机控制任务
  motorControlTaskHandle  = (osThreadId_t)xTaskCreateStatic(
      vMotorControlTask, "motorControl", 512, NULL,
      osPriorityRealtime, motorControlTask_Stack, &motorControlTask_TCB);
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
  // 不启用掉电延迟初始化定时器（功能已停用）
  (void)argument;
  if (modbus1sTimerHandle != NULL) {
    osTimerStart(modbus1sTimerHandle, 1000U);
  }
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
extern volatile uint8_t experiment_event;
extern volatile uint8_t experiment_mode;
extern volatile uint8_t experiment_output_limit;
/**
  * @brief 电机控制任务 (最高实时优先级)
  * @param argument: 未使用
  * @retval None
  * @note 绝对的 10ms 周期执行闭环 PID 计算
  */
void vMotorControlTask(void *argument)
{
  uint32_t PreviousWakeTime = osKernelGetTickCount();
  for(;;)
  {
    // 绝对延时周期 (由 CONTROL_FREQ_HZ 决定), 确保控制环路稳定
    osDelayUntil(PreviousWakeTime + CONTROL_PERIOD_TICKS);
    PreviousWakeTime = osKernelGetTickCount();

    // 获取互斥锁，保护串口指令与本地闭环计算的数据安全
    if (xMotorDataMutexHandle != NULL) {
      osMutexAcquire(xMotorDataMutexHandle, osWaitForever);
      Motor_Control_Loop(); // 执行电机位置/速度内环闭环计算并输出PWM
      osMutexRelease(xMotorDataMutexHandle);
    }
  }
}

/**
  * @brief Modbus 通信解析任务 (高优先级)
  * @param argument: 未使用
  * @retval None
  * @note 等待串口接收中断通过消息队列传递来的 Modbus 帧，解析后更新寄存器或执行相应操作
  */
void vModbusCommTask(void *argument)
{
  MODBUS_FrameTypeDef frame;

  for(;;)
  {
    if (modbusRxQueueHandle == NULL)
    {
      osDelay(1);
      continue;// 如果消息队列未创建成功，等待后重试
    }

    if (osMessageQueueGet(modbusRxQueueHandle, &frame, NULL, osWaitForever) == osOK) {
      if (frame.length > MODBUS_BUF_SIZE)
      {
        continue;// 如果接收到的帧长度超过缓冲区大小，丢弃该帧
      }

      if (xMotorDataMutexHandle != NULL) {
        if (osMutexAcquire(xMotorDataMutexHandle, 10) == osOK) { // 尝试拿锁，超时 10ms
          Modbus_Event(&frame); 
          osMutexRelease(xMotorDataMutexHandle);
        }
      }
    }
  }
}

/**
 * @brief 传感器处理任务 (普通优先级)
 *
 * 每 10ms 启动一次 ADC DMA 单次采集 (4 通道 × 15 样本),
 * DMA 正常模式: 传输完成自动停止, ISR 快照 → 滑动窗口均值滤波 → 写入 ADC_HallValue[],
 * 最后调用 Stop_DMA 为下一轮准备干净状态。
 */
void vSensorProcessTask(void *argument)
{
  uint16_t local[60];
  uint8_t i, j;
  uint32_t sum;

  for(;;)
  {
    // 启动单次 DMA 采集 (4 通道 × 15 样本)
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_NativeValue, 60);

    // 等待 DMA 完成 (超时 50ms 为安全兜底)
    if (osSemaphoreAcquire(Sem_ADC_Done, 50) != osOK) {
      HAL_ADC_Stop_DMA(&hadc1);
      continue;
    }

    // 拷贝快照到本地栈
    for (i = 0; i < 60; i++) {
      local[i] = ADC_Snapshot[i];
    }

    // 滑动窗口均值: 每通道 15 样本直接求和取平均
    // 加锁写入, 保证控制任务和监控任务读到一致的 ADC_HallValue[]
    if (xMotorDataMutexHandle != NULL) {
      osMutexAcquire(xMotorDataMutexHandle, osWaitForever);
    }
    for (j = 0; j < 4; j++) {
      sum = 0;
      for (i = 0; i < 15; i++) {
        sum += local[4 * i + j];
      }
      ADC_HallValue[j] = (uint16_t)(sum / 15);
    }
    if (xMotorDataMutexHandle != NULL) {
      osMutexRelease(xMotorDataMutexHandle);
    }

    // 停止 ADC + DMA, 为下一轮 Start_DMA 准备干净的状态
    HAL_ADC_Stop_DMA(&hadc1);

    osDelay(10);  // 10ms 后进入下一轮采集
  }
}

//static char pcwritebuff[256]; // 用于打印任务栈大小的缓冲区
/**
  * @brief 系统监视及低频定时任务 (低优先级)
  * @param argument: 未使用
  * @retval None
  * @note 用于处理100ms周期的状态监控和实验数据输出
  */
void vSystemMonitorTask(void *argument)
{
  uint32_t PreviousWakeTime = osKernelGetTickCount();
  int pl_was_lost = 0;  // 掉电恢复标志
  for(;;)
  {
    // 100ms周期监控，同时输出减速比实验CSV数据
    osDelayUntil(PreviousWakeTime + 100);
    PreviousWakeTime = osKernelGetTickCount();

    // 12V 主电掉电检测 (ADC2 IN4, 100ms 轮询 + 2 次消抖 = 200ms)
    if (!PL_Is12VAlive()) {
      if (!pl_was_lost) {
        PL_EmergencyStop();  // 首次掉电: 刹停 + 关 LED
        pl_was_lost = 1;
      }
      continue;  // 掉电中, 跳过 Modbus 计时
    }

    // 12V 恢复
    if (pl_was_lost) {
      PL_Resume();         // 开 LED
      pl_was_lost = 0;
    }
      // 先输出实验事件，再输出同周期数据，便于上位机准确切分采集区间
      if (experiment_event == 1U) {
        printf("EXPERIMENT_START,1,%u,%u\r\n",
               (unsigned int)experiment_output_limit,
               (unsigned int)experiment_mode);
        experiment_event = 0U;
      } else if (experiment_event == 2U) {
        printf("EXPERIMENT_STOP\r\n");
        experiment_event = 0U;
      }
      Print_Motor_Status(0, 5); // 输出食指：电机编号,电机累计圈数,关节角度
    }

  }


/**
 * @brief 1s 定时器回调：设置 Modbus 周期性标志（事件驱动替代计时累加）
 */
void Modbus1sTimerCallback(void *argument)
{
  (void)argument;
  // 通过定时器触发 Modbus 周期事件，保留简单标志兼容现有逻辑
  modbus.Host_time_flag = 1;
}


  /**
    * @brief 掉电恢复延迟初始化定时器回调
    * @param argument: 未使用
    * @retval None
    */
  /* 掉电恢复功能已停用：保留空回调以防外部引用 */
  void PowerLossInitTimerCallback(void *argument)
  {
    (void)argument;
    // 功能被禁用 -- 原实现已注释
  }

/**
 * @brief ADC DMA 传输完成回调 — 快照 + 信号量唤醒
 *
 * DMA 正常模式 (非循环), 完成 60 次半字传输后硬件自动停止。
 * 此时 ADC_NativeValue[] 已稳定, 拷贝到 ADC_Snapshot(CCM) 无竞态。
 * 频率 = 任务频率 (10ms → 100 Hz), 不会淹没调度器。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    for (int i = 0; i < 60; i++) {
        ADC_Snapshot[i] = ADC_NativeValue[i];
    }
    osSemaphoreRelease(Sem_ADC_Done);
}

/* USER CODE END Application */

