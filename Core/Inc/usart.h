/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "wrist.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
extern uint8_t RES1;
extern uint8_t RES2;	
/*��λ�����������ݰ���ʽ��
	FF     01       AA          FE
   ��ͷ   ����1  ��������ģʽ   ��β
*/

//����3��6��Ӧ�����������ͽ��ܽṹ��
//����ṹ��û�õ����е�bug
/**************************************/
// Ӧ��֡�ṹ��
typedef struct {
    uint8_t header1;    // ֡ͷ1
    uint8_t header2;    // ֡ͷ2
    uint8_t data_length; // ���ݳ���
    uint8_t id_address; // ID��ַ
    uint8_t cmd_type;   // ָ������
    uint16_t register_addr; // �Ĵ�����ַ
    uint8_t status;     // ״̬��
    uint8_t data[MAX_FRAME_LENGTH]; // ���ݶ�
    uint8_t check_sum; // У���
} UART_Response_Frame;
/*************************************/

// ����״̬��״̬����
typedef enum {
    RX_HEADER1,
    RX_HEADER2,
    RX_LENGTH,
    RX_ID,
    RX_CMD,
    RX_REG_LOW,
    RX_REG_HIGH,
    RX_DATA,
    RX_CHECKSUM
} UART_RxState;

// ȫ�ֽ���״̬����
extern UART_RxState rx_state ;
extern UART_RxState rx_state1 ;
extern uint8_t rx_buffer[MAX_FRAME_LENGTH]; //���ݽ��ܻ�����
extern uint8_t rx_buffer1[MAX_FRAME_LENGTH];
extern uint8_t rx_index ;  
extern uint8_t rx_index1 ;
extern uint8_t rx_data_length ;
extern uint8_t rx_data_length1 ;
//û�õ����е�bug
extern UART_Response_Frame response_frame;
extern UART_Response_Frame response_frame1;
//
//���ݽ�����ɱ�־λ
extern uint8_t received;
extern uint8_t received1;

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
//uint8_t Serial_GetRxFlag(void);//ʶ������жϱ�־λ	
//*�������ܺ���*	
void hhSerialSendByte(uint8_t Byte);//����һ���ֽ�
void hhSerialSendArray(uint8_t *Array, uint16_t Length);//��������
void hhSerialSendString(char * mString);//�����ַ���
uint32_t Serial_Pow(uint32_t X, uint32_t Y);
void hhSerial_SendNumber(uint32_t Number, uint8_t Length);//��������
//��������غ���
//�����ʹ�ӡӦ��֡����
//void print_response_frame(UART_Response_Frame *frame);
//void parse_response_frame(UART_Response_Frame *frame, uint8_t *buffer);
//��������3��6�����ж�
void init_uart_receive(UART_HandleTypeDef *huart);
void init_uart_receive1(UART_HandleTypeDef *huart);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

