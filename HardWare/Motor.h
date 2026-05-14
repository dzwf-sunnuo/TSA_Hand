#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"

#define Motor_Num 4
#define Hall_Num 4

#define Real_OneTurn 365.0f
#define Real_MaxSpeed 50.0f

#define CalibrationLEN 2

typedef struct _Motor_Struct
{
    float TargetSpeed;
    float CurrentSpeed;
    float TargetAngle;
    float CurrentAngle;
    float TargetPosition;
    float CurrentPosition;
    uint8_t IOFlag;
    float Turns;
} Motor_Struct;

typedef struct _PID_Increment_Struct
{
    float Kp, Ki, Kd;
    float Error_Last1;
    float Error_Last2;
    float Out_Last;
} PID_Increment_Struct;

typedef struct {
    TIM_HandleTypeDef* pwm_tim;
    uint32_t pwm_ch1;
    uint32_t pwm_ch2;
    TIM_HandleTypeDef* enc_tim;
} Motor_HW_Config;

extern const Motor_HW_Config motor_hw_map[Motor_Num];

extern Motor_Struct motor[Motor_Num];
extern PID_Increment_Struct PID_Speed[Motor_Num];
extern PID_Increment_Struct PID_Angle[Motor_Num];
extern PID_Increment_Struct PID_Position[Motor_Num];
extern uint16_t ADC_HallValue[Hall_Num];
extern float TargetPosition[Hall_Num];

void Motor_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
float PID_Increment(PID_Increment_Struct *PID, float Current, float Target);
void Set_Motor(uint8_t i, int16_t set_speed, uint8_t flag);
float Get_Speed(uint8_t i);
float Get_Angle(uint8_t i);
float lineInterp(float xa[], float ya[], int length, float data, int flag);

#endif
