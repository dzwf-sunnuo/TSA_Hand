#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"

#define Motor_Num 4//定义电机个数为4个，1&2：MCP内收外展；3：PIP伸直；4：总弯曲
#define Hall_Num 4//定义关节霍尔个数为4个，1：MCP内收外展；2；MCP屈伸；3：PIP屈伸；4：DIP屈伸

//13mm大朗络电机3500RPM
//#define Real_OneTurn 359.2//单位-度°
//#define Real_MaxSpeed 36//单位-RPM，乘以0~100才是实际的最大速度

//10mm小朗络电机5000RPM
#define Real_OneTurn 365//单位-度°
#define Real_MaxSpeed 50//单位-RPM，乘以0~100才是实际的最大速度

//************二分查找、线性插值相关变量************
#define CalibrationLEN 2//测到标定数据集JointAngle-AD的长度，比如0~90度测了5个点，该值就是5
//*电机结构体*
typedef struct _Motor_Struct
{
	float TargetSpeed;//目标转速,单位为RPM，范围是120(20%)~690(100%)
	float CurrentSpeed;//实际转速,单位为RPM，范围是120(20%)~690(100%)
	float TargetAngle;//目标电机轴角度,单位为 °
	float CurrentAngle;//实际电机轴角度,单位为 °
	float TargetPosition;//目标关节位置,用HALL_AD值一一对应，单位为“1”
	float CurrentPosition;//实际关节位置,用HALL_AD值一一对应，单位为“1”
	uint8_t IOFlag;	
	float Turns;//转过的圈数
}Motor_Struct;
//*PID计算结构体*
typedef struct _PID_Increment_Struct
{
	float Kp,Ki,Kd;//三个PID参数
	float Error_Last1;//上次误差
	float Error_Last2;//上上次误差
	float Out_Last;//上次输出
}PID_Increment_Struct;

//************电机PID相关全局变量**************
extern Motor_Struct motor[Motor_Num];
extern PID_Increment_Struct PID_Speed[Motor_Num];
extern PID_Increment_Struct PID_Angle[Motor_Num];
extern PID_Increment_Struct PID_Position[Motor_Num];
extern uint16_t ADC_HallValue[Hall_Num];
extern float TargetPosition[Hall_Num];

//************电机相关函数**************
void Motor_Init(void);//初始化
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);//10ms中断执行PID计算
float PID_Increment(PID_Increment_Struct *PID, float Current, float Target);//增量式PID计算公式
void Set_Motor(uint8_t i, int16_t set_speed, uint8_t flag);//电机执行
float Get_Speed(uint8_t i);//各电机出轴当前转速获取
float Get_Angle(uint8_t i);//各电机出轴当前转过的角度获取
//*************插值相关函数***************
//JointAngle-AD，data为待查找的数据；flag为查找类型标志（10为x.data且x降序；11为x.data且x升序）（20为y.data且y降序；21为y.data且y升序）
float lineInterp(float xa[], float ya[], int length, float data, int flag);//二分查找+线性插值函数


#endif
