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
#include "sensor_feedback.h"
#include "can_telemetry.h"
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
uint16_t ADC_NativeValue[60] = {0}; // DMA直接写入的60点原始缓冲，必须保留在普通SRAM，禁止迁入CCMRAM
__CCM_RAM_DATA uint16_t ADC_Snapshot[60] = {0};    // 采集停止后由任务复制的稳定原始快照

// 以下仅迁移CPU使用的对象句柄；对象本体的动态堆及DMA缓冲不改变位置。
__CCM_RAM_DATA osMutexId_t xMotorDataMutexHandle;
const osMutexAttr_t xMotorDataMutex_attributes = {
  .name = "xMotorDataMutex"
};
__CCM_RAM_DATA osSemaphoreId_t Sem_ADC_Done;  // DMA 单次采集完成信号
__CCM_RAM_DATA osMessageQueueId_t modbusRxQueueHandle;
// UART 发送完成信号量（用于替换 busy-wait）
__CCM_RAM_DATA osSemaphoreId_t uartTxSemHandle;

// 1s 周期 Modbus 定时器（替代原先的累加计时方式）
__CCM_RAM_DATA osTimerId_t modbus1sTimerHandle;
const osTimerAttr_t modbus1sTimer_attributes = {
  .name = "modbus1sTimer"
};

// LED 闪烁定时器 (RS485 收发触发双闪, 50ms 周期, 一次性)
__CCM_RAM_DATA osTimerId_t ledBlinkTimerHandle;
const osTimerAttr_t ledBlinkTimer_attributes = {
  .name = "ledBlinkTimer"
};

// 注：暂时停用掉电延迟初始化定时器
// osTimerId_t powerLossInitTimerHandle;
// const osTimerAttr_t powerLossInitTimer_attributes = {
//   .name = "powerLossInitTimer"
// };

// CAN使用静态任务存储，不额外消耗已有15KB动态堆。
__CCM_RAM_DATA static StaticTask_t canTelemetryTask_TCB;
__CCM_RAM_DATA static StackType_t canTelemetryTask_Stack[512];
__CCM_RAM_DATA osThreadId_t canTelemetryTaskHandle;
__CCM_RAM_DATA volatile uint32_t sensorAcquisitionOverruns = 0U;

__CCM_RAM_DATA osThreadId_t motorControlTaskHandle;
__CCM_RAM_DATA static StaticTask_t motorControlTask_TCB;
__CCM_RAM_DATA static StackType_t motorControlTask_Stack[512];  // 512 words = 2KB
const osThreadAttr_t motorControlTask_attributes = {
  .name = "motorControlTask",
  .priority = (osPriority_t) osPriorityRealtime,
};

__CCM_RAM_DATA osThreadId_t modbusCommTaskHandle;
const osThreadAttr_t modbusCommTask_attributes = {
  .name = "modbusCommTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

__CCM_RAM_DATA osThreadId_t sensorProcessTaskHandle;
const osThreadAttr_t sensorProcessTask_attributes = {
  .name = "sensorProcessTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

__CCM_RAM_DATA osThreadId_t systemMonitorTaskHandle;
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
// MX_FREERTOS_Init：创建现有控制任务与静态CAN反馈任务及同步对象
// 参数：无
// 返回值：无
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
  canTelemetryTaskHandle = (osThreadId_t)xTaskCreateStatic(
      vCanTelemetryTask, "canTelemetry", 512, NULL,
      osPriorityBelowNormal, canTelemetryTask_Stack, &canTelemetryTask_TCB);
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

// vApplicationGetIdleTaskMemory：覆盖CMSIS弱定义，为空闲任务提供CCMRAM静态存储
// 参数：ppxIdleTaskTCBBuffer - 控制块输出；ppxIdleTaskStackBuffer - 栈地址输出；pulIdleTaskStackSize - 栈深输出（StackType_t个数）
// 返回值：无；栈深保持configMINIMAL_STACK_SIZE，禁止将空闲任务栈用作DMA缓冲
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    __CCM_RAM_DATA static StaticTask_t idleTask_TCB;
    __CCM_RAM_DATA static StackType_t idleTask_Stack[configMINIMAL_STACK_SIZE]
        __attribute__((aligned(portBYTE_ALIGNMENT)));
    *ppxIdleTaskTCBBuffer = &idleTask_TCB;
    *ppxIdleTaskStackBuffer = idleTask_Stack;
    *pulIdleTaskStackSize = (uint32_t)configMINIMAL_STACK_SIZE;
}

// vApplicationGetTimerTaskMemory：覆盖CMSIS弱定义，为软件定时器服务任务提供CCMRAM静态存储
// 参数：ppxTimerTaskTCBBuffer - 控制块输出；ppxTimerTaskStackBuffer - 栈地址输出；pulTimerTaskStackSize - 栈深输出（StackType_t个数）
// 返回值：无；现有定时器回调只访问CPU状态，栈深与优先级保持原配置，不把栈地址交给DMA
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
    StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    __CCM_RAM_DATA static StaticTask_t timerTask_TCB;
    __CCM_RAM_DATA static StackType_t timerTask_Stack[configTIMER_TASK_STACK_DEPTH]
        __attribute__((aligned(portBYTE_ALIGNMENT)));
    *ppxTimerTaskTCBBuffer = &timerTask_TCB;
    *ppxTimerTaskStackBuffer = timerTask_Stack;
    *pulTimerTaskStackSize = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}

extern volatile uint8_t experiment_event;
extern volatile uint8_t experiment_mode;
extern volatile uint8_t experiment_output_limit;
// vMotorControlTask：按累计绝对截止时间运行100Hz控制，不等待传感器采集
// 参数：argument - 未使用
// 返回值：无，持续运行；超期跳过旧周期，不突发补算
void vMotorControlTask(void *argument)
{
  (void)argument;
  uint32_t next_tick = osKernelGetTickCount() + CONTROL_PERIOD_TICKS;
  for (;;) {
    osDelayUntil(next_tick);
    next_tick += CONTROL_PERIOD_TICKS;
    if (xMotorDataMutexHandle != NULL) {
      osMutexAcquire(xMotorDataMutexHandle, osWaitForever);
      Motor_Control_Loop();
      osMutexRelease(xMotorDataMutexHandle);
    }
    uint32_t now_tick = osKernelGetTickCount();
    if ((int32_t)(now_tick - next_tick) >= 0) {
      next_tick += ((now_tick - next_tick) / CONTROL_PERIOD_TICKS + 1U) * CONTROL_PERIOD_TICKS;
    }
  }
}

/**
  * @brief Modbus 通信解析任务 (高优先级)
  * @param argument: 未使用
  * @retval None
  * @note 等待串口接收中断通过消息队列传递来的 Modbus 帧，解析后更新寄存器或执行相应操作
  */
// vModbusCommTask：串行处理完整MODBUS帧，寄存器由协议函数原子提交
// 参数：argument - 未使用
// 返回值：无，持续处理请求
void vModbusCommTask(void *argument)
{
  (void)argument;
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

      // 寄存器由协议函数短临界区原子提交，串口发送等待不再占用电机锁。
      Modbus_Event(&frame);
    }
  }
}

// vSensorProcessTask：每10毫秒完成一批ADC与三路SPI并发布完整缓存
// 参数：argument - 未使用
// 返回值：无；绝对周期累计调度，超期计数并跳过旧周期
void vSensorProcessTask(void *argument)
{
  (void)argument;
  const uint32_t period_ticks = osKernelGetTickFreq() / SENSOR_ACQUISITION_HZ;
  uint32_t next_tick = osKernelGetTickCount();
  for (;;) {
    SensorFeedback_Acquire(HAL_GetTick());
    next_tick += period_ticks;
    uint32_t now_tick = osKernelGetTickCount();
    if ((int32_t)(now_tick - next_tick) >= 0) {
      sensorAcquisitionOverruns++;
      next_tick += ((now_tick - next_tick) / period_ticks + 1U) * period_ticks;
    }
    osDelayUntil(next_tick);
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
    osDelayUntil(PreviousWakeTime + 250);
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
      /*if (experiment_event == 1U) {
        printf("EXPERIMENT_START,1,%u,%u\r\n",
               (unsigned int)experiment_output_limit,
               (unsigned int)experiment_mode);
        experiment_event = 0U;
      } else if (experiment_event == 2U) {
        printf("EXPERIMENT_STOP\r\n");
        experiment_event = 0U;
      }*/
      Print_Motor_Status(tactile_motor_indices[0], 101); // 默认输出中指motor1的FF数据帧
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

// HAL_ADC_ConvCpltCallback：在DMA满块时立即冻结ADC，再通知采集任务
// 参数：hadc - 完成转换的ADC句柄，仅处理ADC1
// 返回值：无；不在中断里复制缓冲或进行滤波
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) SensorFeedback_AdcFinishedFromISR(1U);
}

// HAL_ADC_ErrorCallback：结束错误批次并唤醒任务执行有界恢复
// 参数：hadc - 报错的ADC句柄，ADC2掉电检测维持原逻辑
// 返回值：无
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) SensorFeedback_AdcFinishedFromISR(0U);
}

/* USER CODE END Application */

