#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"

/* 系统常量定义 */
#define Motor_Num 4             // 系统管理的电机总数 (食指、中指、无名指、小指)
#define Hall_Num 4              // 关节霍尔传感器数量
#define CalibrationLEN 2        // 传感器标定点数量 (用于线性插值)

/* 电机物理特性常量 */
#define Real_OneTurn 365.0f     // 减速后输出轴转动一圈对应的编码器计数值(已换算为角度)
#define Real_MaxSpeed 50.0f      // 电机运行允许的最大速度？
/**
 * @brief 电机状态结构体
 * 存储单个电机的实时反馈与目标设定值
 */
typedef struct _Motor_Struct
{
    float TargetSpeed;          // 目标转速
    float CurrentSpeed;         // 当前实时转速 (由编码器计算)
    float TargetAngle;          // 目标出轴角度
    float CurrentAngle;         // 当前出轴角度 (由编码器累加)
    float TargetPosition;       // 目标关节位置 (由线性插值得到的 ADC 目标值)
    float CurrentPosition;      // 当前关节位置 (实时 ADC 滤波值)
    uint8_t IOFlag;             // 启停控制位 (1: 运行, 0: 停止/刹车)
    float Turns;                // 累计转动圈数 (预留)
} Motor_Struct;

/**
 * @brief 增量式 PID 控制器结构体
 */
typedef struct _PID_Increment_Struct
{
    float Kp, Ki, Kd;           // PID 三项增益参数
    float Error_Last1;          // 上一次误差 E(k-1)
    float Error_Last2;          // 上上次误差 E(k-2)
    float Out_Last;             // 上一次的控制器输出 U(k-1)
} PID_Increment_Struct;

/**
 * @brief 电机硬件配置映射
 * 用于将逻辑电机编号与具体的定时器通道关联
 */
typedef struct {
    TIM_HandleTypeDef* pwm_tim; // PWM 输出定时器句柄
    uint32_t pwm_ch1;           // PWM 通道 1 (正转控制)
    uint32_t pwm_ch2;           // PWM 通道 2 (反转控制)
    TIM_HandleTypeDef* enc_tim; // 编码器输入定时器句柄
} Motor_HW_Config;

/* 全局外部变量声明 */
extern const Motor_HW_Config motor_hw_map[Motor_Num]; // 硬件映射表

extern Motor_Struct motor[Motor_Num];                // 电机状态数组
extern PID_Increment_Struct PID_Speed[Motor_Num];      // 速度环 PID
extern PID_Increment_Struct PID_Angle[Motor_Num];      // 出轴角度环 PID
extern PID_Increment_Struct PID_Position[Motor_Num];   // 关节位置环 PID
extern uint16_t ADC_HallValue[Hall_Num];              // 关节传感器滤波后的 ADC 值
extern float TargetPosition[Hall_Num];               // 关节目标位置 (调试用)

/* 核心 API 函数原型 */

/**
 * @brief 电机系统初始化
 * 开启 PWM、编码器外设并初始化 PID 参数
 */
void Motor_Init(void);

/**
 * @brief 电机控制闭环主逻辑
 * 建议在 10ms 周期任务中调用，执行层叠 PID 计算
 */
void Motor_Control_Loop(void);

/**
 * @brief 后台计时逻辑
 * 在 1ms 定时任务中调用，处理系统心跳及 Modbus 延时
 */
void Modbus_Timer_Loop(void);

/**
 * @brief 增量式 PID 核心算法
 */
float PID_Increment(PID_Increment_Struct *PID, float Current, float Target);

/**
 * @brief 底层电机驱动输出
 * @param i 电机索引
 * @param set_speed 控制增量 (-1000 到 1000)
 * @param flag 启停标志
 */
void Set_Motor(uint8_t i, int16_t set_speed, uint8_t flag);

/**
 * @brief 获取当前转速
 */
float Get_Speed(uint8_t i);

/**
 * @brief 获取当前角度
 */
float Get_Angle(uint8_t i);

/**
 * @brief 线性插值函数
 * 用于将目标物理角度映射为非线性的传感器 ADC 值
 */
float lineInterp(float xa[], float ya[], int length, float data, int flag);

#endif
