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
uint16_t ADC_NativeValue[80] = {0}; // DMA 目标缓冲区 (单次模式)
__CCM_RAM_DATA uint16_t ADC_Snapshot[80] = {0};    // 完成回调中拷贝的快照

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

// 注：暂时停用掉电延迟初始化定时器
// osTimerId_t powerLossInitTimerHandle;
// const osTimerAttr_t powerLossInitTimer_attributes = {
//   .name = "powerLossInitTimer"
// };

osThreadId_t motorControlTaskHandle;
// 电机控制任务使用静态分配: TCB + 栈全部放入 CCMRAM (零等待)
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
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  modbusRxQueueHandle = osMessageQueueNew(MODBUS_RX_QUEUE_LENGTH, sizeof(MODBUS_FrameTypeDef), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // 静态创建电机控制任务: TCB + 栈全在 CCMRAM, 无堆分配碎片风险
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
/**
  * @brief 电机控制任务 (最高实时优先级)
  * @param argument: 未使用
  * @retval None
  * @note 绝对的 10ms 周期执行闭环 PID 计算
  */
__CCM_RAM_TEXT void vMotorControlTask(void *argument)
{
  uint32_t PreviousWakeTime = osKernelGetTickCount();
 // uint32_t i = 0,j = 0; 
  for(;;)
  {
    // 绝对延时周期 (由 CONTROL_FREQ_HZ 决定), 确保控制环路稳定
    osDelayUntil(PreviousWakeTime + CONTROL_PERIOD_TICKS);
    PreviousWakeTime = osKernelGetTickCount();

    // 获取互斥锁，保护串口指令与本地闭环计算的数据安全
    if (xMotorDataMutexHandle != NULL) {
     // i = osKernelGetTickCount();
      osMutexAcquire(xMotorDataMutexHandle, osWaitForever);
      Motor_Control_Loop(); // 执行电机位置/速度内环闭环计算并输出PWM
      // 掉电保存相关功能已停用，如需恢复请取消注释并启用 timer
      // PL_SaveAngles();        // 将当前角度写入备份寄存器 (双槽交替, ~30μs)
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

/* ================================================================
 *  快速排序 (uint16_t)
 *
 *  标准 Hoare 分区方案。对 20 个元素的小数组, 快速排序
 * 
 *
 *  调用: quicksort(arr, 0, n-1)
 * ================================================================ */

static void swap_u16(uint16_t *a, uint16_t *b)
{
    uint16_t t = *a;
    *a = *b;
    *b = t;
}

/**
 * @brief 分区: 以最右元素为 pivot, 小于 pivot 的放左边, 大于的放右边
 * @return pivot 最终位置
 */
static int partition(uint16_t *arr, int left, int right)
{
    uint16_t pivot = arr[right];  // 选最右元素为基准
    int i = left - 1;             // i 指向"小于区"的尾部

    for (int j = left; j < right; j++) {
        if (arr[j] <= pivot) {
            i++;
            swap_u16(&arr[i], &arr[j]);
        }
    }
    swap_u16(&arr[i + 1], &arr[right]);  // pivot 归位
    return i + 1;
}

/**
 * @brief 快速排序 (递归)
 * @param arr   待排序数组
 * @param left  左边界 (含)
 * @param right 右边界 (含)
 */
static void quicksort(uint16_t *arr, int left, int right)
{
    if (left >= right) return;

    int pivot_idx = partition(arr, left, right);

    quicksort(arr, left, pivot_idx - 1);   // 排左边
    quicksort(arr, pivot_idx + 1, right);  // 排右边
}

/**
 * @brief 传感器处理任务 (普通优先级)
 *
 * 每 10ms 启动一次 ADC DMA 单次采集 (4 通道 × 20 样本),
 * 等待完成回调 → 快速排序 + 裁剪均值滤波 → 写入 ADC_HallValue[]。
 *
 * DMA 为单次模式, 仅在采集期间运行 (~152μs), 其余时间 ADC 空闲。
 */
void vSensorProcessTask(void *argument)
{
  uint16_t local[80];
  uint16_t sorted[20];
  uint8_t i, j;
  uint32_t sum;

  for(;;)
  {
    // 启动单次 DMA 采集 (4 通道 × 20 样本 ≈ 152μs)
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_NativeValue, 80);

    // 等待 DMA 完成 (超时 50ms 为安全兜底)
    if (osSemaphoreAcquire(Sem_ADC_Done, 50) != osOK) {
      HAL_ADC_Stop_DMA(&hadc1);
      continue;
    }

    // 拷贝快照到本地栈
    for (i = 0; i < 80; i++) {
      local[i] = ADC_Snapshot[i];
    }

    for (j = 0; j < 4; j++) {

      for (i = 0; i < 20; i++) {
        sorted[i] = local[4 * i + j];
      }

      quicksort(sorted, 0, 19);

      // 裁剪均值: 去头尾各 4, 中间 12 取平均
      sum = 0;
      for (i = 4; i < 16; i++) {
        sum += sorted[i];
      }
      ADC_HallValue[j] = (uint16_t)(sum / 12);
    }

    osDelay(5);  // 5ms 后进入下一轮采集
  }
}

static char pcwritebuff[256]; // 用于打印任务栈大小的缓冲区
/**
  * @brief 系统监视及低频定时任务 (低优先级)
  * @param argument: 未使用
  * @retval None
  * @note 用于处理 200ms 其他逻辑
  */
void vSystemMonitorTask(void *argument)
{
  uint32_t PreviousWakeTime = osKernelGetTickCount();
  //int i = 3; // 监视电机 i-1 的状态
  for(;;)
  {
    // 200ms 的 Modbus 定时节拍器
    osDelayUntil(PreviousWakeTime + 200); 
    /*打印电机1的目标速度，实际速度，目标位置，实际位置*/
    vTaskList(pcwritebuff);
    printf("Task List:\r\n%s\r\n", pcwritebuff);
   //Print_Motor_Status(i, 1); // 打印速度状态
    Modbus_Timer_Loop(); // 更新 Modbus 超时接收和 1s 定时发送计时器，其实超时接收已经被IDLE中断回调处理了
    PreviousWakeTime = osKernelGetTickCount();

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

/* USER CODE END Application */

