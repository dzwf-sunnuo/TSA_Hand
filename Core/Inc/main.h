/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
// CCMRAM为64KB CPU专用数据RAM，起址0x10000000；ADC/UART等DMA缓冲必须留在普通SRAM。
// __CCM_RAM_DATA把全局或静态变量放入.ccmram，启动代码在进入main前从Flash复制初值。
// 未显式初始化的变量也由该段的零初值镜像初始化；不需要在业务代码中另行清零。
// 下方TEXT宏仅保留旧接口兼容：STM32F407的CCMRAM不能执行指令，禁止用于函数。
#define __CCM_RAM_TEXT __attribute__((section(".ccmram.text")))   // 历史兼容定义，本项目不使用
#define __CCM_RAM_DATA __attribute__((section(".ccmram")))        // 仅用于CPU访问的全局或静态数据
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
