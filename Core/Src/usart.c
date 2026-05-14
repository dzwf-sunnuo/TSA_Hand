/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "rs485.h"
#include "wrist.h"
//*********����1��2��Ӧ485ͨ�ű���**********
uint8_t RES1;
uint8_t RES2;

extern volatile uint8_t wrist_enabled;       
//*********����3��6��Ӧ����������**********
UART_RxState rx_state= RX_HEADER1 ;
UART_RxState rx_state1= RX_HEADER1 ;
uint8_t rx_index ;
uint8_t rx_index1 ;
uint8_t rx_data_length;
uint8_t rx_data_length1;
//uint8_t rx_byte;
uint8_t rx_buffer[MAX_FRAME_LENGTH] ;
uint8_t rx_buffer1[MAX_FRAME_LENGTH] ;
uint8_t received;
uint8_t received1;
uint8_t ByteMotor1;
uint8_t ByteMotor2;
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//�жϻص������������жϷ����󣬰ѽ��յ�������RES����modbus.rcbuf[]
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//USART1�����жϣ�����1��485�����ϸ��ӻ��ķ���ֵ��1��485
//	if (huart == &huart1){
//		if(modbus1.reflag == 1)  //�����ݰ����ڴ���
// 	    {
//			return ;
// 		}		
// 	    modbus1.rcbuf[modbus1.recount++] = RES1;
// 		modbus1.timout = 0;
// 		if(modbus1.recount == 1)  //�Ѿ��յ��˵ڶ����ַ�����
// 		{
// 		  modbus1.timrun = 1;  //����modbus��ʱ����ʱ
// 		}		
//		HAL_UART_Receive_IT(&huart1, (uint8_t *)&RES1, 1);//���´򿪴���1�жϽ��գ�Ϊ�´ν���������׼��
//	}
	//USART2�����жϣ�����2��485������PC���������Ķ�����ţ�?2��485
	if (huart == &huart2){
		if(modbus2.reflag == 1)  //�����ݰ����ڴ���
 	    {
      HAL_UART_Receive_IT(&huart2, (uint8_t *)&RES2, 1);
			return ;
 		}		
    if(modbus2.recount < sizeof(modbus2.rcbuf))
    {
      modbus2.rcbuf[modbus2.recount++] = RES2;
      modbus2.timout = 0;
      if(modbus2.recount == 1)  //�Ѿ��յ��˵ڶ����ַ�����
      {
        modbus2.timrun = 1;  //����modbus��ʱ����ʱ
      }
    }
    else
    {
      modbus2.recount = 0;
      modbus2.timrun = 0;
      modbus2.timout = 0;
    }
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&RES2, 1);//���´򿪴���2�жϽ��գ�Ϊ�´ν���������׼��

	}
    //USART3�����жϣ����������Ƹ˵��?1����ֵ
	if (huart == &huart3)
		{
			HAL_UART_Receive_IT(&huart3, &ByteMotor1, 1); //�ŵ�main��ȥд
		    static uint8_t rx_state = RX_HEADER1;
			uint8_t rx_byte = ByteMotor1;
			// �Ӵ��ڽ���һ���ֽ�
		switch(rx_state) {
			case RX_HEADER1:
				if(rx_byte == 0xAA) {
					rx_index = 0;
					rx_buffer[rx_index++] = rx_byte;
					rx_state = RX_HEADER2;               
				}
            break;
            
			case RX_HEADER2:
				if(rx_byte == 0x55) {
					rx_state = RX_LENGTH;
					rx_buffer[rx_index++] = rx_byte;
				} 
				else {
					rx_state = RX_HEADER1; // ֡ͷ�������¿�ʼ
				}
            break;
            
			case RX_LENGTH:
				rx_data_length = rx_byte;
				rx_state = RX_ID;
				rx_buffer[rx_index++] = rx_byte;
            break;
            
			case RX_ID:
				rx_state = RX_CMD;
				rx_buffer[rx_index++] = rx_byte;
            break;
            
			case RX_CMD:
				rx_state = RX_REG_LOW;
				rx_buffer[rx_index++] = rx_byte;
            break;
            
			case RX_REG_LOW:
				rx_state = RX_REG_HIGH;
				rx_buffer[rx_index++] = rx_byte;
			break;
            
			case RX_REG_HIGH:
				rx_buffer[rx_index++] = rx_byte;
				if(rx_data_length > 3) { // ���������?
					rx_state = RX_DATA;
				} 
				else {
                rx_state = RX_CHECKSUM;
				}
            break;            
            
			case RX_DATA:
				rx_buffer[rx_index++] = rx_byte;
				if(rx_index >= rx_data_length + 4) { // ���ݽ������?
					rx_state = RX_CHECKSUM;
				}
            break;
            
			case RX_CHECKSUM:
				// ������У��ͣ��������?
				rx_buffer[rx_index++] = rx_byte;           
				// ��֤У���?
				uint8_t checksum = calculate_checksum(&rx_buffer[2], rx_data_length+2);
				if(checksum == rx_byte) {
                // У�����ȷ������Ӧ���?
					received  = 1; 
//                parse_response_frame(&response_frame, rx_buffer);
//                print_response_frame(&response_frame);
				} 
				else {
//					printf("У��ʹ���\r\n");//������
				}           
				// ����״̬��
				rx_state = RX_HEADER1;
            break;
    }
  }
  //USART6�����жϣ����������Ƹ˵��?2����ֵ
  if(huart == &huart6)
	  {
		  HAL_UART_Receive_IT(&huart6, &ByteMotor2, 1); //�ŵ�main��ȥд
		  static uint8_t rx_state1 = RX_HEADER1;
		  uint8_t rx_byte1 = ByteMotor2;
		  // �Ӵ��ڽ���һ���ֽ�
		  switch(rx_state1) {
			  case RX_HEADER1:
				  if(rx_byte1 == 0xAA) {
					  rx_index1 = 0;
					  rx_buffer1[rx_index1++] = rx_byte1;
					  rx_state1 = RX_HEADER2;               
				  }
			  break;
				  
              case RX_HEADER2:
                  if(rx_byte1 == 0x55) {
                      rx_state1 = RX_LENGTH;
                      rx_buffer1[rx_index1++] = rx_byte1;
                  } 
				  else {
                      rx_state1 = RX_HEADER1; // ֡ͷ�������¿�ʼ
                  }
              break;
            
			  case RX_LENGTH:
				  rx_data_length1 = rx_byte1;
				  rx_state1 = RX_ID;
				  rx_buffer1[rx_index1++] = rx_byte1;
              break;
            
			  case RX_ID:
				  rx_state1 = RX_CMD;
			      rx_buffer1[rx_index1++] = rx_byte1;
              break;
            
              case RX_CMD:
                  rx_state1 = RX_REG_LOW;
                  rx_buffer1[rx_index1++] = rx_byte1;
              break;
                  
              case RX_REG_LOW:
                  rx_state1 = RX_REG_HIGH;
                  rx_buffer1[rx_index1++] = rx_byte1;
              break;
                  
              case RX_REG_HIGH:
                  rx_buffer1[rx_index1++] = rx_byte1;
		      	  if(rx_data_length1 > 3) { // ���������?
					  rx_state1 = RX_DATA;
                  } 
		      	  else {
                      rx_state1 = RX_CHECKSUM;
                  }
              break;            
                  
              case RX_DATA:
                  rx_buffer1[rx_index1++] = rx_byte1;
                  if(rx_index1 >= rx_data_length1 + 4) { // ���ݽ������?
                      rx_state1 = RX_CHECKSUM;
                  }
              break;
                  
              case RX_CHECKSUM:
				  // ������У��ͣ��������?
				  rx_buffer1[rx_index1++] = rx_byte1;
				  // ��֤У���?
				  uint8_t checksum = calculate_checksum(&rx_buffer1[2], rx_data_length1+2);
				  if(checksum == rx_byte1) {
				  // У�����ȷ������Ӧ���?
				  received1  = 1; 
//                parse_response_frame(&response_frame, rx_buffer);
//                print_response_frame(&response_frame);
				  } 
				  else {
//					  printf("У��ʹ���\r\n");//������
				  } 
				  // ����״̬��
				  rx_state1 = RX_HEADER1;
				  break;
		  }
	  }
}

void init_uart_receive(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_IT(huart, &ByteMotor1, 1);
}
void init_uart_receive1(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_IT(huart, &ByteMotor2, 1);
}
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
/* USER CODE END 1 */
