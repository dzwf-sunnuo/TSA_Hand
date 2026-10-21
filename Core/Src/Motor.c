#include "Motor.h"
#include "stdio.h"
#include <math.h>
#include "adc.h"
#include "usart.h"
#include "rs485.h"
#include "rs485_crc.h"
#include "admittance.h"
#include "Tactile_Sensor.h"
#include "angle_codec.h"
#include "sensor_feedback.h"

/* 本板三路触觉导纳：中指、无名指、小指对应motor1、motor2、motor3 */
#define TACTILE_FINGER_COUNT 3U
#define TACTILE_FORCE_LIMIT_N 15.0f
#define MODE6_RESTART_FLAG 0x01U
#define MODE7_RESTART_FLAG 0x02U

extern volatile uint8_t admittance_mode_restart; // 上位机重新写入模式6/7的事件标志
extern uint16_t ADC_Snapshot[60]; // ADC DMA 完成后的原始采样快照，按四通道交错存放，每通道十五个样本

__CCM_RAM_DATA static Admittance_Ctrl adm_ctrl[TACTILE_FINGER_COUNT];
__CCM_RAM_DATA static uint8_t mode6_force_locked[TACTILE_FINGER_COUNT] = {0};
__CCM_RAM_DATA static uint8_t mode7_force_locked[TACTILE_FINGER_COUNT] = {0};

// 唯一的电机/触觉通道对应表，采集、控制、调试与CAN共同使用。
const uint8_t tactile_motor_indices[TACTILE_FINGER_COUNT] = {1U, 2U, 3U};
static const char *const tactile_sensor_names[TACTILE_FINGER_COUNT] = {
    "Middle", "Ring", "Little"
};

/* 堵转保护 */
#define STALL_TIMEOUT_SECONDS 5.0f   // 堵转判定时间 (秒)
#define STALL_TIMEOUT_TICKS   ((uint16_t)(STALL_TIMEOUT_SECONDS * CONTROL_FREQ_HZ))
#define STALL_ANGLE_THRESHOLD  (Real_OneTurn * 1.02f)  // 1 圈 = ~7.3 编码器计数

/**
 * @brief 硬件配置映射表
 * 
 * 将每个电机对应的 PWM 定时器、通道以及编码器定时器进行解耦映射。
 * 顺序对应：电机 0 (食指), 电机 1 (中指), 电机 2 (无名指), 电机 3 (小指)。
 * 两次板子接口不一致
 */

 /*这个是老接线板（需要自己焊接编码器的那一版)
const Motor_HW_Config motor_hw_map[Motor_Num] = {
    {&htim5, TIM_CHANNEL_1, TIM_CHANNEL_2, &htim2}, // 电机 0
    {&htim5, TIM_CHANNEL_3, TIM_CHANNEL_4, &htim3}, // 电机 1
    {&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, &htim4}, // 电机 2
    {&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, &htim8}  // 电机 3
};
*/

// 这个是新接线板260506（编码器接口直接焊在排针上），立创EDA里面所有的电机M+M-还有编码器全都乱套了，还没改，改了把这行删了哈哈
const Motor_HW_Config motor_hw_map[Motor_Num] = {
    {&htim1, TIM_CHANNEL_2, TIM_CHANNEL_1, &htim3}, // 电机 0
    {&htim5, TIM_CHANNEL_4, TIM_CHANNEL_3, &htim2}, // 电机 1
    {&htim1, TIM_CHANNEL_4, TIM_CHANNEL_3, &htim4}, // 电机 2
    {&htim5, TIM_CHANNEL_2, TIM_CHANNEL_1, &htim8}  // 电机 3
};



/* 全局变量定义 */
Motor_Struct motor[Motor_Num] __CCM_RAM_DATA = {0};          // 电机状态结构体数组
PID_Increment_Struct PID_Speed[Motor_Num] __CCM_RAM_DATA = {0};    // 速度环 PID 状态
PID_Increment_Struct PID_Position[Motor_Num] __CCM_RAM_DATA = {0}; // 关节位置环 PID 状态
PID_Increment_Struct PID_Angle[Motor_Num] __CCM_RAM_DATA = {0};    // 出轴角度环 PID 状态

__CCM_RAM_DATA uint16_t ADC_HallValue[Hall_Num] = {0}; // CPU使用的滤波结果，非DMA目标，放入CCMRAM

/**
 * @brief 计算常数预推导，老电机要全部*20
 *
 * 所有公式统一从 CONTROL_FREQ_HZ 派生, 修改频率只需改 Motor.h 一个地方
 *
 * SPEED_CONV_FACTOR:
 *   公式: (控制频率Hz * 60s * 修正系数1.15) / (减速比4 * 编码器线数1024 * 4倍频)
 *   例:   100Hz → (100*60*1.15)/16384 ≈ 0.42 rpm/pulse
 * ANGLE_CONV_FACTOR:
 *   公式: (360度 * 修正系数1.15) / (减速比4 * 编码器线数1024 * 4倍频)
 *   与频率无关，保持不变
 */
#define SPEED_CONV_FACTOR ((float)(CONTROL_FREQ_HZ) * 60.0f * 1.15f / (4.0f * 1024.0f * 4.0f))
#define ANGLE_CONV_FACTOR (360.0f * 1.15f / (4.0f * 1024.0f * 4.0f))

/* 标定数据集：用于 lineInterp 函数将角度映射为 ADC 目标值 */
__CCM_RAM_DATA static float JointAngle[4][CalibrationLEN] = {{0.0f, 90.0f}, {0.0f, 90.0f}, {0.0f, 90.0f}, {0.0f, 90.0f}} ;
__CCM_RAM_DATA static float ADValue[4][CalibrationLEN] = {
    {1570.0f, 2570.0f}, // 关节 1
    {2475.0f, 1435.0f}, // 关节 2
    {2500.0f, 1450.0f}, // 关节 3
    {2565.0f, 1540.0f}  // 关节 4
} ;

__CCM_RAM_DATA static int InterpolationFlag[4] = {11, 11, 11, 11} ; // 插值搜索方向标志

/* PID 初始参数 */
__CCM_RAM_DATA float Kp_Position[Hall_Num]  = {1.5f, 0.5f, 0.5f, 0.5f} ;
__CCM_RAM_DATA float Ki_Position[Hall_Num] = {0.03f, 0.03f, 0.03f, 0.03f} ;  // 小积分消除静差, 克服摩擦力
__CCM_RAM_DATA float Kd_Position[Hall_Num] = {0.1f, 0.0f, 0.0f, 0.0f} ;

__CCM_RAM_DATA float Kp_Angle[Motor_Num] = {0.5f, 0.7f, 0.7f, 0.7f} ;
__CCM_RAM_DATA float Ki_Angle[Motor_Num] = {0.0f, 0.0f, 0.0f, 0.0f} ;
__CCM_RAM_DATA float Kd_Angle[Motor_Num] = {0.0f, 0.0f, 0.0f, 0.0f} ;

__CCM_RAM_DATA float Kp_Speed[Motor_Num] = {0.3f, 0.3f, 0.3f, 0.3f} ;
__CCM_RAM_DATA float Ki_Speed[Motor_Num] = {0.1f, 0.1f, 0.1f, 0.1f} ;
__CCM_RAM_DATA float Kd_Speed[Motor_Num] = {0.0f, 0.0f, 0.0f, 0.0f} ;

// Motor_Init：初始化电机与原有PID，并绑定本板三指触觉采集
// 参数：无；须在TIM13启动后调用
// 返回值：无
void Motor_Init(void)
{
    // 循环开启硬件外设
    for (int i = 0; i < Motor_Num; i++) {
        HAL_TIM_PWM_Start(motor_hw_map[i].pwm_tim, motor_hw_map[i].pwm_ch1);
        HAL_TIM_PWM_Start(motor_hw_map[i].pwm_tim, motor_hw_map[i].pwm_ch2);
        HAL_TIM_Encoder_Start(motor_hw_map[i].enc_tim, TIM_CHANNEL_ALL);
        HAL_TIM_Base_Start_IT(motor_hw_map[i].enc_tim); // 开启溢出中断以保证计数连续
    }

    // 开启 10ms 控制律计算定时器 (仅作为计数参考，不再使用其 IT 中断)
    HAL_TIM_Base_Start(&htim6); 
    HAL_TIM_Base_Start(&htim7); 

    // 初始化 PID 变量
    for (int i = 0; i < Motor_Num; i++) {
        motor[i].CurrentAngle = 0.0f;
        motor[i].CurrentSpeed = 0.0f;

        // 角度环: 输出 TargetSpeed, 范围 ±12750 (50rpm × 255%)
        PID_Angle[i].Kp = Kp_Angle[i];
        PID_Angle[i].Ki = Ki_Angle[i];
        PID_Angle[i].Kd = Kd_Angle[i];
        PID_Angle[i].Error_Last1 = 0.0f;
        PID_Angle[i].Error_Last2 = 0.0f;
        PID_Angle[i].Out_Last = 0.0f;
        PID_Angle[i].OutMin = -12750.0f;
        PID_Angle[i].OutMax =  12750.0f;

        // 速度环: 输出 PWM 占空比, 范围 ±1000 (匹配 int16_t 安全范围)
        PID_Speed[i].Kp = Kp_Speed[i];
        PID_Speed[i].Ki = Ki_Speed[i];
        PID_Speed[i].Kd = Kd_Speed[i];
        PID_Speed[i].Error_Last1 = 0.0f;
        PID_Speed[i].Error_Last2 = 0.0f;
        PID_Speed[i].Out_Last = 0.0f;
        PID_Speed[i].OutMin = -1000.0f;
        PID_Speed[i].OutMax =  1000.0f;
    }

    for (int i = 0; i < Hall_Num; i++) {
        PID_Position[i].Kp = Kp_Position[i];
        PID_Position[i].Ki = Ki_Position[i];
        PID_Position[i].Kd = Kd_Position[i];
        PID_Position[i].Error_Last1 = 0.0f;
        PID_Position[i].Error_Last2 = 0.0f;
        PID_Position[i].Out_Last = 0.0f;
        PID_Position[i].OutMin = -12750.0f;
        PID_Position[i].OutMax =  12750.0f;
    }

    // 将默认 PID 参数回写到 Reg[], 保证上位机 Modbus 读到的值正确
    Reg[PID_REG_KP_POS] = (uint16_t)(Kp_Position[0] * 100.0f);
    Reg[PID_REG_KI_POS] = (uint16_t)(Ki_Position[0] * 100.0f);
    Reg[PID_REG_KD_POS] = (uint16_t)(Kd_Position[0] * 100.0f);
    Reg[PID_REG_KP_ANG] = (uint16_t)(Kp_Angle[0] * 100.0f);
    Reg[PID_REG_KI_ANG] = (uint16_t)(Ki_Angle[0] * 100.0f);
    Reg[PID_REG_KD_ANG] = (uint16_t)(Kd_Angle[0] * 100.0f);
    Reg[PID_REG_KP_SPD] = (uint16_t)(Kp_Speed[0] * 100.0f);
    Reg[PID_REG_KI_SPD] = (uint16_t)(Ki_Speed[0] * 100.0f);
    Reg[PID_REG_KD_SPD] = (uint16_t)(Kd_Speed[0] * 100.0f);

    // 导纳控制参数默认值 (非对称: 接触省力, 松手快回弹)
    // 缩放改为 ×0.001, 获得更精细的低刚度范围
    // Reg[ADM_REG_K] = 接触刚度 ×1000
    // Reg[ADM_REG_B] = 回弹刚度 ×1000 (1000=1.0 N/圈)
    Reg[ADM_REG_K]    = 0x0020; // K_contact = 0.032
    Reg[ADM_REG_B]    = 1000; // K_return  = 1.000
    Reg[ADM_REG_DAMP] = 0x0010; // B_damp = 0.016

    // 初始化三套独立导纳控制器和触觉传感器
    for (uint8_t i = 0U; i < TACTILE_FINGER_COUNT; i++) {
        Admittance_Init(&adm_ctrl[i], 0.0f, 0.016f, 0.032f, 70.0f);
    }
    SensorFeedback_Init();
}

/**
 * @brief 定时器溢出中断回调函数 (核心控制环)
 * 
 * @param htim 定时器句柄。
 *   - htim7 (1ms): 处理 Modbus 通信超时和 1s 定时发送。
 *   - htim6 (10ms): 执行 4 路电机的级联 PID 计算（位置/角度环 -> 速度环）。
 */
void Modbus_Timer_Loop(void)
{
    /* 100ms 步进的后台计时逻辑 (例如 1s 定时发送) */
    modbus.Host_Sendtime++;
    if (modbus.Host_Sendtime >= 10) { // 
        modbus.Host_time_flag = 1; // 1秒定时到达标志
        modbus.Host_Sendtime = 0;  // 自动复位计数器

        /*某些东西。。。*/


    }
}

/**
 * @brief 从 Modbus 寄存器 Reg[] 同步 PID 参数到全部 PID 结构体
 *
 * 缩放: Reg 值 ÷ 100.0f → PID 浮点参数
 * 所有 4 电机共享相同参数, 同时更新
 */
void PID_UpdateGains(void)
{
    float kp_pos = (float)Reg[PID_REG_KP_POS] * 0.01f;
    float ki_pos = (float)Reg[PID_REG_KI_POS] * 0.01f;
    float kd_pos = (float)Reg[PID_REG_KD_POS] * 0.01f;
    float kp_ang = (float)Reg[PID_REG_KP_ANG] * 0.01f;
    float ki_ang = (float)Reg[PID_REG_KI_ANG] * 0.01f;
    float kd_ang = (float)Reg[PID_REG_KD_ANG] * 0.01f;
    float kp_spd = (float)Reg[PID_REG_KP_SPD] * 0.01f;
    float ki_spd = (float)Reg[PID_REG_KI_SPD] * 0.01f;
    float kd_spd = (float)Reg[PID_REG_KD_SPD] * 0.01f;

    for (int i = 0; i < Motor_Num; i++) {
        PID_Angle[i].Kp    = kp_ang;
        PID_Angle[i].Ki    = ki_ang;
        PID_Angle[i].Kd    = kd_ang;
        PID_Speed[i].Kp    = kp_spd;
        PID_Speed[i].Ki    = ki_spd;
        PID_Speed[i].Kd    = kd_spd;
    }
    for (int i = 0; i < Hall_Num; i++) {
        PID_Position[i].Kp = kp_pos;
        PID_Position[i].Ki = ki_pos;
        PID_Position[i].Kd = kd_pos;
    }
}

// Motor_Control_Loop：读取成品反馈缓存，执行四路电机状态更新与原有闭环控制
// 参数：无
// 返回值：无
void Motor_Control_Loop(void)
{
    SensorFeedbackSnapshot_t feedback;
    SensorFeedback_ReadSnapshot(&feedback);
    uint32_t now_ms = HAL_GetTick();
    bool adc_fresh = SensorFeedback_IsFresh(feedback.adc_valid, feedback.adc_update_tick, now_ms);
    if (adc_fresh) {
        for (uint8_t channel = 0U; channel < Hall_Num; channel++) {
            ADC_HallValue[channel] = feedback.adc_mean[channel];
        }
    }
    /* [2] 10ms 电机控制闭环逻辑 */
        // 仅在 Modbus 写入了 PID 寄存器时才更新 (避免每 10ms 无效刷新)
        if (pid_params_dirty) {
            PID_UpdateGains();
            pid_params_dirty = 0;
        }
        uint8_t mode = Reg[4] >> 8;     // 从 Modbus 寄存器 Reg[4] 高 8 位获取运动模式
        uint8_t io_flag = Reg[4] & 0xFF; // 从 Modbus 寄存器 Reg[4] 低 8 位获取启停标志

        // SPI由独立采集任务执行，本函数不再等待ADC/SPI或操作传感器片选。

        // 上位机再次写入模式6/7时，解除对应模式的三路超限锁定并重置导纳状态
        if ((admittance_mode_restart & MODE6_RESTART_FLAG) != 0U) {
            for (uint8_t i = 0U; i < TACTILE_FINGER_COUNT; i++) {
                mode6_force_locked[i] = 0U;
                Admittance_Reset(&adm_ctrl[i]);
            }
            admittance_mode_restart &= (uint8_t)~MODE6_RESTART_FLAG;
        }
        if ((admittance_mode_restart & MODE7_RESTART_FLAG) != 0U) {
            for (uint8_t i = 0U; i < TACTILE_FINGER_COUNT; i++) {
                mode7_force_locked[i] = 0U;
                Admittance_Reset(&adm_ctrl[i]);
            }
            admittance_mode_restart &= (uint8_t)~MODE7_RESTART_FLAG;
        }

        /* 堵转保护状态 (跨调用保持) */
        __CCM_RAM_DATA static uint16_t stall_timer[Motor_Num] = {0}; // CPU控制状态，非DMA缓冲
        __CCM_RAM_DATA static float stall_target[Motor_Num] = {0};

        for (int i = 0; i < Motor_Num; i++) {
            int8_t tactile_index = Motor_GetTactileChannel((uint8_t)i);
            /* 2.1 状态采集 (已优化：单词读取 delta 确保速度/位置同步) */
            int16_t delta = (int16_t)__HAL_TIM_GET_COUNTER(motor_hw_map[i].enc_tim);
            __HAL_TIM_SET_COUNTER(motor_hw_map[i].enc_tim, 0);

            motor[i].IOFlag = io_flag;
            motor[i].CurrentAngle += (float)delta * ANGLE_CONV_FACTOR;
            // 立即更新掉电保存 CRC：尽量在角度更新后立刻计算 CRC，
            // 以缩短角度采样到掉电保存之间的时间窗口。
            // 注意：每次在此处计算会在 10ms 循环内执行多次（最多 Motor_Num 次），
            // 代价是额外的少量 CPU 开销，但能保证 CRC 数据尽可能接近实时角度。
            motor[i].CurrentSpeed = (float)delta * SPEED_CONV_FACTOR;
            motor[i].CurrentPosition = (float)ADC_HallValue[i];

            // 关节/导纳闭环不能用启动无数据或过期反馈驱动；保留目标和超限锁定等待恢复。
            bool force_missing = (mode == 6U || mode == 7U) && tactile_index >= 0 &&
                !SensorFeedback_IsFresh(feedback.force[tactile_index].valid,
                    feedback.force[tactile_index].last_update_tick, now_ms);
            if (((mode == 5U || mode == 7U) && !adc_fresh) || force_missing) {
                motor[i].TargetSpeed = 0.0f;
                PID_Position[i].Out_Last = 0.0f;
                PID_Position[i].Error_Last1 = PID_Position[i].Error_Last2 = 0.0f;
                PID_Angle[i].Out_Last = 0.0f;
                PID_Angle[i].Error_Last1 = PID_Angle[i].Error_Last2 = 0.0f;
                PID_Speed[i].Out_Last = 0.0f;
                PID_Speed[i].Error_Last1 = PID_Speed[i].Error_Last2 = 0.0f;
                Set_Motor((uint8_t)i, 0, 0);
                continue;
            }

            /* ---- 堵转保护 (仅 Mode 1/3 角度环有效, Mode 5/6 跳过) ---- */
            if (io_flag && (mode == 1 || mode == 3)) {
                // 目标变化 → 重置计时
                if (motor[i].TargetAngle != stall_target[i]) {
                    stall_target[i] = motor[i].TargetAngle;
                    stall_timer[i] = 0;
                }
                // 误差超阈值 → 累加堵转计时
                float err = motor[i].TargetAngle - motor[i].CurrentAngle;
                if (err < 0) err = -err;
                if (err > STALL_ANGLE_THRESHOLD) {
                    if (++stall_timer[i] >= STALL_TIMEOUT_TICKS) {
                        // 堵转确认: 清除目标圈数, PID 自动回零
                        Reg[i] &= 0x00FF;   // 保留低 8 位速度, 圈数清零
                        stall_timer[i] = 0;
                    }
                } else {
                    stall_timer[i] = 0;  // 在接近目标, 重置
                }
            } else {
                stall_timer[i] = 0;
            }

            /* 2.2 外环计算 (位置环/角度环) */
            switch (mode) {
                case 1: // 模式1：正向出轴角度控制
                case 3: // 模式3：反向出轴角度控制
                {
                    float sign = (mode == 1) ? 1.0f : -1.0f;
                    motor[i].TargetAngle = sign * Real_OneTurn * (float)(Reg[i] >> 8);
                    
                    // 软限位：限制最大转动圈数为 40 圈
                    if (motor[i].TargetAngle > Real_OneTurn * 60.0f) motor[i].TargetAngle = Real_OneTurn * 60.0f;
                    if (motor[i].TargetAngle < -Real_OneTurn * 60.0f) motor[i].TargetAngle = -Real_OneTurn * 60.0f;

                    // 计算角度环输出 -> 得到目标速度
                    motor[i].TargetSpeed = PID_Increment(&PID_Angle[i], motor[i].CurrentAngle, motor[i].TargetAngle);
                    
                    // 速度限幅：根据 Reg[i] 低 8 位设定的百分比限制最大速度
                    float max_spd = Real_MaxSpeed * (float)(Reg[i] & 0xFF);
                    if (motor[i].TargetSpeed > max_spd) motor[i].TargetSpeed = max_spd;
                    else if (motor[i].TargetSpeed < -max_spd) motor[i].TargetSpeed = -max_spd;
                    break;
                }
                case 4: // 模式4：编码器清零/紧急停止
                    motor[i].CurrentAngle = 0.0f;
                    motor[i].TargetAngle = 0.0f;
                    motor[i].CurrentSpeed = 0.0f;
                    motor[i].TargetSpeed = 0.0f;
                    break;
                case 5: // 模式5：关节角度插值控制 (Reg[i] 使用四位压缩BCD角度)
                {
                    uint16_t target_tenths = 0U;
                    if (!AngleCodec_DecodeBcdTenths(Reg[i], &target_tenths)) {
                        motor[i].TargetSpeed = 0.0f;
                        PID_Position[i].Out_Last = 0.0f;
                        PID_Position[i].Error_Last1 = 0.0f;
                        PID_Position[i].Error_Last2 = 0.0f;
                        PID_Speed[i].Out_Last = 0.0f;
                        PID_Speed[i].Error_Last1 = 0.0f;
                        PID_Speed[i].Error_Last2 = 0.0f;
                        Set_Motor(i, 0, 0); // 非法BCD立即停止，跳过本周期速度环
                        continue;
                    }

                    float target_joint_angle = (float)target_tenths * 0.1f;
                    if (target_joint_angle < 5.0f) target_joint_angle = 5.0f;
                    if (target_joint_angle > 90.0f) target_joint_angle = 90.0f;

                    // 通过线性插值将目标物理角度转换为目标 ADC 值
                    motor[i].TargetPosition = lineInterp(JointAngle[i], ADValue[i], CalibrationLEN, target_joint_angle, InterpolationFlag[i]);
                    // 位置环计算 -> 得到目标速度
                    motor[i].TargetSpeed = PID_Increment(&PID_Position[i], motor[i].CurrentPosition, motor[i].TargetPosition);

                    // 死区处理：ADC 误差在 5 以内则停止, 同时复位 PID 状态防止重启时跳变
                    if (PID_Position[i].Error_Last1 <= 5.0f && PID_Position[i].Error_Last1 >= -5.0f) {
                        motor[i].TargetSpeed = 0.0f;
                        PID_Position[i].Out_Last = 0.0f;     // 复位积分累加
                        PID_Position[i].Error_Last1 = 0.0f;  // 清零历史误差
                        PID_Position[i].Error_Last2 = 0.0f;
                    }
                    // 安全保护：若电机转动超过 40 圈则强制停止以防拉断
                    if (motor[i].CurrentAngle < -60.0f * Real_OneTurn || motor[i].CurrentAngle > 60.0f * Real_OneTurn) motor[i].TargetSpeed = 0.0f;
                    break;
                }
                case 6: // 模式6：本板三指分别执行输出轴角度导纳控制
                {
                    // 中指、无名指、小指使用各自的传感器和导纳控制器
                    if (tactile_index >= 0) {
                        if (mode6_force_locked[tactile_index] == 0U &&
                            SensorFeedback_IsFresh(feedback.force[tactile_index].valid,
                                feedback.force[tactile_index].last_update_tick, now_ms)) {
                            // 使用本控制周期统一采集的力值，避免在导纳模式中重复进行SPI读取
                            float force = feedback.force[tactile_index].f_sum;

                            // 单指超过15N时只锁定该手指，并将其安全目标设为0圈
                            if (force > TACTILE_FORCE_LIMIT_N) {
                                mode6_force_locked[tactile_index] = 1U;
                                Reg[i] = 0x0064U;
                                Admittance_Reset(&adm_ctrl[tactile_index]);
                            } else {
                                // 基准圈数从对应寄存器高8位读出
                                float base_turns = (float)(Reg[i] >> 8);
                                adm_ctrl[tactile_index].B = (float)Reg[ADM_REG_DAMP] * 0.001f;
                                adm_ctrl[tactile_index].K = (force > 0.1f)
                            ? (float)Reg[ADM_REG_K] * 0.001f    // K_contact
                            : (float)Reg[ADM_REG_B] * 0.001f;   // K_return
                                float delta_turns = Admittance_Update(&adm_ctrl[tactile_index], force, 0.01f);
                                motor[i].TargetAngle = Real_OneTurn * (base_turns - delta_turns);
                                if (motor[i].TargetAngle > Real_OneTurn * 40.0f)
                                    motor[i].TargetAngle = Real_OneTurn * 40.0f;
                                if (motor[i].TargetAngle < -Real_OneTurn * 40.0f)
                                    motor[i].TargetAngle = -Real_OneTurn * 40.0f;
                            }
                        }

                        // 锁定期间维持0圈目标，直到上位机再次写入0x0601
                        if (mode6_force_locked[tactile_index] != 0U) {
                            motor[i].TargetAngle = 0.0f;
                        }
                    }

                    // 未绑定触觉的motor0保留原目标，本板三指使用对应的新目标
                    motor[i].TargetSpeed = PID_Increment(&PID_Angle[i],
                        motor[i].CurrentAngle, motor[i].TargetAngle);
                    float max_spd = Real_MaxSpeed * (float)(Reg[i] & 0xFF);
                    if (motor[i].TargetSpeed > max_spd)  motor[i].TargetSpeed = max_spd;
                    else if (motor[i].TargetSpeed < -max_spd) motor[i].TargetSpeed = -max_spd;
                    break;
                }
                case 7: // 模式7：本板三指分别执行关节角度导纳控制
                {
                    // 中指、无名指、小指使用各自传感器，motor0执行普通位置控制
                    if (tactile_index >= 0) {
                        uint16_t base_tenths = 0U;
                        if (!AngleCodec_DecodeBcdTenths(Reg[i], &base_tenths)) {
                            motor[i].TargetSpeed = 0.0f;
                            PID_Position[i].Out_Last = 0.0f;
                            PID_Position[i].Error_Last1 = 0.0f;
                            PID_Position[i].Error_Last2 = 0.0f;
                            PID_Speed[i].Out_Last = 0.0f;
                            PID_Speed[i].Error_Last1 = 0.0f;
                            PID_Speed[i].Error_Last2 = 0.0f;
                            Set_Motor(i, 0, 0); // 非法BCD立即停止，跳过本周期速度环
                            continue;
                        }

                        if (mode7_force_locked[tactile_index] == 0U &&
                            SensorFeedback_IsFresh(feedback.force[tactile_index].valid,
                                feedback.force[tactile_index].last_update_tick, now_ms)) {
                            // 使用本控制周期统一采集的力值，避免在导纳模式中重复进行SPI读取
                            float force = feedback.force[tactile_index].f_sum;

                            // 单指超过15N时只锁定该手指，并将其安全目标设为5.0°
                            if (force > TACTILE_FORCE_LIMIT_N) {
                                mode7_force_locked[tactile_index] = 1U;
                                Reg[i] = 0x0050U;
                                Admittance_Reset(&adm_ctrl[tactile_index]);
                            } else {
                                float base_angle = (float)base_tenths * 0.1f;
                                if (base_angle < 5.0f) base_angle = 5.0f;
                                if (base_angle > 90.0f) base_angle = 90.0f;

                                adm_ctrl[tactile_index].x_max = 40.0f;
                                adm_ctrl[tactile_index].B = (float)Reg[ADM_REG_DAMP] * 0.001f;
                                adm_ctrl[tactile_index].K = (force > 0.1f)
                                    ? (float)Reg[ADM_REG_K] * 0.001f
                                    : (float)Reg[ADM_REG_B] * 0.001f;

                                float delta_deg = Admittance_Update(&adm_ctrl[tactile_index], force, 0.01f);
                                float target_angle = base_angle - delta_deg;
                                if (target_angle < 5.0f) target_angle = 5.0f;
                                if (target_angle > 90.0f) target_angle = 90.0f;
                                motor[i].TargetPosition = lineInterp(JointAngle[i], ADValue[i],
                                    CalibrationLEN, target_angle, InterpolationFlag[i]);
                            }
                        }

                        // 锁定期间持续执行5.0°安全目标，直到上位机再次写入0x0701
                        if (mode7_force_locked[tactile_index] != 0U) {
                            float target_angle = 5.0f;
                            if (target_angle < 5.0f)  target_angle = 5.0f;
                            if (target_angle > 90.0f) target_angle = 90.0f;
                            motor[i].TargetPosition = lineInterp(JointAngle[i], ADValue[i],
                                CalibrationLEN, target_angle, InterpolationFlag[i]);
                        }
                    } else {
                        // 未绑定触觉的motor0使用模式5逻辑，从四位压缩BCD读取目标角度
                        uint16_t target_tenths = 0U;
                        if (!AngleCodec_DecodeBcdTenths(Reg[i], &target_tenths)) {
                            motor[i].TargetSpeed = 0.0f;
                            PID_Position[i].Out_Last = 0.0f;
                            PID_Position[i].Error_Last1 = 0.0f;
                            PID_Position[i].Error_Last2 = 0.0f;
                            PID_Speed[i].Out_Last = 0.0f;
                            PID_Speed[i].Error_Last1 = 0.0f;
                            PID_Speed[i].Error_Last2 = 0.0f;
                            Set_Motor(i, 0, 0); // 非法BCD立即停止，跳过本周期速度环
                            continue;
                        }

                        float target_angle = (float)target_tenths * 0.1f;
                        if (target_angle < 5.0f)  target_angle = 5.0f;
                        if (target_angle > 90.0f) target_angle = 90.0f;
                        motor[i].TargetPosition = lineInterp(JointAngle[i], ADValue[i],
                            CalibrationLEN, target_angle, InterpolationFlag[i]);
                    }
                    // 全电机位置环闭环 (ADC 反馈)
                    motor[i].TargetSpeed = PID_Increment(&PID_Position[i],
                        motor[i].CurrentPosition, motor[i].TargetPosition);

                    // 死区处理: ADC 误差 ≤5 则停止
                    if (PID_Position[i].Error_Last1 <= 5.0f && PID_Position[i].Error_Last1 >= -5.0f) {
                        motor[i].TargetSpeed = 0.0f;
                        PID_Position[i].Out_Last = 0.0f;
                        PID_Position[i].Error_Last1 = 0.0f;
                        PID_Position[i].Error_Last2 = 0.0f;
                    }
                    // 安全保护: 编码器超 60 圈则停止
                    if (motor[i].CurrentAngle < -60.0f * Real_OneTurn || motor[i].CurrentAngle > 60.0f * Real_OneTurn)
                        motor[i].TargetSpeed = 0.0f;
                    break;
                }
                default:
                    motor[i].TargetSpeed = 0.0f;
                    break;
            }

            /* 2.3 内环计算 (速度环) */
            int16_t set_speed = (int16_t)PID_Increment(&PID_Speed[i], motor[i].CurrentSpeed, motor[i].TargetSpeed);
            
            // 输出限幅：PWM 对应占空比范围限制在 [-1000, 1000]
            if (set_speed > 1000) set_speed = 1000;
            else if (set_speed < -1000) set_speed = -1000;

            /* 2.4 硬件执行 */
            Set_Motor(i, set_speed, motor[i].IOFlag);
        }

        /* CRC 更新已移到每个电机角度更新之后，以缩短角度变化到掉电保存之间的延迟 */
}

/**
 * @brief 硬件驱动执行函数
 * 
 * 将 PID 计算出的速度值转换为具体的 PWM 比较寄存器值。
 * 
 * @param i 电机索引 (0-3)。
 * @param set_speed 控制增量值 (-1000 到 1000)。
 * @param flag 启停标志 (0: 停止, 非0: 运行)。
 */
void Set_Motor(uint8_t i, int16_t set_speed, uint8_t flag)
{
    if (i >= Motor_Num) return;
    if (flag == 0) set_speed = 0; // 若 flag 为 0，则强制停止输出
    
    // 中心对齐 PWM 计算：基准值 500，set_speed/2 决定差分驱动方向
    __HAL_TIM_SET_COMPARE(motor_hw_map[i].pwm_tim, motor_hw_map[i].pwm_ch1, 500 + set_speed / 2);
    __HAL_TIM_SET_COMPARE(motor_hw_map[i].pwm_tim, motor_hw_map[i].pwm_ch2, 500 - set_speed / 2);
}

/**
 * @brief 获取单个电机的当前转速 (RPM)
 */
float Get_Speed(uint8_t i)
{
    if (i >= Motor_Num) return 0.0f;
    int16_t zj = (int16_t)__HAL_TIM_GET_COUNTER(motor_hw_map[i].enc_tim);
    __HAL_TIM_SET_COUNTER(motor_hw_map[i].enc_tim, 0);
    return (float)zj * SPEED_CONV_FACTOR;
}

/**
 * @brief 获取单个电机的当前出轴角度 (不清除计数器)
 */
float Get_Angle(uint8_t i)
{
    if (i >= Motor_Num) return 0.0f;
    int16_t zj = (int16_t)__HAL_TIM_GET_COUNTER(motor_hw_map[i].enc_tim);
    return (float)zj * ANGLE_CONV_FACTOR;
}

/**
 * @brief 线性插值算法 (用于传感器映射)
 * 
 * 在标定好的数据点之间进行二分查找，并根据斜率计算中间值。
 * 
 * @param xa[] 标定点的 X 轴数组 (例如：角度)。
 * @param ya[] 标定点的 Y 轴数组 (例如：ADC码值)。
 * @param length 标定点数量。
 * @param data 待转换的输入值。
 * @param flag 搜索方向标志 (10: X降序, 11: X升序)。
 * @return float 返回插值计算后的 Y 值。
 */
float lineInterp(float xa[], float ya[], int length, float data, int flag)
{
    int low = 0;
    int high = length - 1;
    int mid = 0;
    
    if (flag == 10) { // X 数据为降序排列
        if (data >= xa[low]) return ya[low];
        if (data <= xa[high]) return ya[high];
        while (high - low > 1) {
            mid = (high + low) / 2;
            if (xa[mid] < data) high = mid;
            else if (xa[mid] > data) low = mid;
            else return ya[mid];
        }
    } else if (flag == 11) { // X 数据为升序排列
        if (data <= xa[low]) return ya[low];
        if (data >= xa[high]) return ya[high];
        while (high - low > 1) {
            mid = (high + low) / 2;
            if (xa[mid] < data) low = mid;
            else if (xa[mid] > data) high = mid;
            else return ya[mid];
        }
    } else {
        return 0.0f; // 未知标志，返回 0
    }
    
    // 标准线性插值公式: y = y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return ya[low] + (ya[high] - ya[low]) * (data - xa[low]) / (xa[high] - xa[low]);
}

/**
 * @brief 增量式 PID 算法实现
 * 
 * 相比于位置式 PID，增量式 PID 不需要积分项累加，对系统抗干扰能力和切换平滑性更好。
 * 
 * @param PID PID 状态结构体指针。
 * @param Current 当前反馈值。
 * @param Target 目标设定值。
 * @return float PID 控制器的输出增量。
 */
float PID_Increment(PID_Increment_Struct *PID, float Current, float Target)
{
    float err = Target - Current;
    float proportion = err - PID->Error_Last1;
    float differential = err - 2.0f * PID->Error_Last1 + PID->Error_Last2;

    float out = PID->Out_Last + PID->Kp * proportion + PID->Ki * err + PID->Kd * differential;

    // 抗积分饱和: Out_Last 必须钳位在输出限幅范围内, 否则堵转时
    // 积分项持续累加, float→int16_t 转换时溢出回绕导致电机反转
    if (out > PID->OutMax)  out = PID->OutMax;
    if (out < PID->OutMin)  out = PID->OutMin;

    PID->Error_Last2 = PID->Error_Last1;
    PID->Error_Last1 = err;
    PID->Out_Last = out;

    return out;
}

// Print_Motor_Status：按指定格式输出电机、关节、触觉或 ADC 原始采样数据
// 参数：motor_index - 电机索引，范围0～3，101仅支持有触觉传感器的1～3；param - 输出类型，1～5为现有状态，6为 ADC 原始值，101为关节角度、法向力与切向力之比、指尖合力数据帧
// 返回值：无
void Print_Motor_Status(uint8_t motor_index, uint8_t param)
{
    if (motor_index >= Motor_Num) return;
    SensorFeedbackSnapshot_t feedback;
    SensorFeedback_ReadSnapshot(&feedback);
    int8_t tactile_index = Motor_GetTactileChannel(motor_index);
    if (param == 1) {// 打印速度信息
        printf("Motor %d Target Speed: %.2f RPM, Actual Speed: %.2f RPM\r\n", motor_index + 1, motor[motor_index].TargetSpeed, motor[motor_index].CurrentSpeed);
    } else if (param == 2) {// 打印角度信息
        printf("Motor %d Target Angle: %.2f deg, Actual Angle: %.2f deg\r\n", motor_index + 1, motor[motor_index].TargetAngle , motor[motor_index].CurrentAngle);
    }else if(param == 3){//打印当前关节角度 (0-90°), 由 ADC 值通过标定表换算
        float adc_current = motor[motor_index].CurrentPosition;
        float adc_0  = ADValue[motor_index][0];  // 0° 对应 ADC
        float adc_90 = ADValue[motor_index][1];  // 90° 对应 ADC
        float angle_current = (adc_current - adc_0) / (adc_90 - adc_0) * 90.0f;
        printf("%.2f \n",
               angle_current);
    } else if (param == 4) {// 打印触觉传感器三维力
        if (tactile_index >= 0) {
            printf("[%s] Fx:%.1fN Fy:%.1fN Fz:%.1fN |F|=%.1fN\r\n",
                tactile_sensor_names[tactile_index],
                feedback.force[tactile_index].fx, feedback.force[tactile_index].fy,
                feedback.force[tactile_index].fz, feedback.force[tactile_index].f_sum);
        } else {
            printf("Motor %d has no tactile sensor\r\n", motor_index + 1);
        }
    } else if (param == 5) {// 输出减速比实验CSV：电机编号,电机累计圈数,关节角度
        float motor_turns = motor[motor_index].CurrentAngle / Real_OneTurn;
        float adc_current = motor[motor_index].CurrentPosition;
        float adc_0 = ADValue[motor_index][0];
        float adc_90 = ADValue[motor_index][1];
        float joint_angle = (adc_current - adc_0) / (adc_90 - adc_0) * 90.0f;

        printf("%u,%.4f,%.2f\r\n",
               (unsigned int)(motor_index + 1U), motor_turns, joint_angle);
    } else if (param == 6) {// 打印对应关节通道的 ADC 原始整数值，不进行均值滤波或角度换算
        // 一轮采集包含十五组四通道数据，最后一组从下标56开始；读取完成快照中的最后一个对应通道样本
        uint16_t adc_native = ADC_Snapshot[56U + motor_index];
        printf("%u\r\n", (unsigned int)adc_native);
    } else if (param == 101) {// 仅输出 FF、三个数值和 FE，以逗号分隔并换行结束
        // motor0未绑定触觉，不输出提示文字，避免混入数据帧
        if (tactile_index < 0) return;

        // 根据对应关节的零度与九十度标定值换算角度，并限制到0～90度
        float adc_current = motor[motor_index].CurrentPosition;
        float adc_0 = ADValue[motor_index][0];
        float adc_90 = ADValue[motor_index][1];
        float joint_angle = (adc_current - adc_0) / (adc_90 - adc_0) * 90.0f;
        if (joint_angle < 0.0f) joint_angle = 0.0f;
        if (joint_angle > 90.0f) joint_angle = 90.0f;

        // 按Z轴为法向、XY平面为切向计算力的大小；切向力为零时用-1标记无效比值，避免除零
        float force_x = feedback.force[tactile_index].fx;
        float force_y = feedback.force[tactile_index].fy;
        float normal_force = fabsf(feedback.force[tactile_index].fz);
        float tangential_force = sqrtf(force_x * force_x + force_y * force_y);
        float force_ratio = (tangential_force > 0.0f) ? normal_force / tangential_force : -1.0f;

        // 三个数值依次为关节角度、法向力/切向力、指尖三维合力（单位为牛顿），不添加名称或单位文字
        printf("FF,%.2f,%.2f,%.2f,FE\r\n",
               joint_angle, force_ratio, feedback.force[tactile_index].f_sum);
    }
}//用于调试，定期打印电机状态信息，观察 PID 收敛情况和系统响应特性

// Motor_GetTactileChannel：从集中映射表查找指定电机对应的触觉缓存下标
// 参数：motor_index - 原有电机编号0～3
// 返回值：0～2表示触觉通道，-1表示本板未绑定
int8_t Motor_GetTactileChannel(uint8_t motor_index)
{
    for (uint8_t channel = 0U; channel < TACTILE_FINGER_COUNT; channel++) {
        if (tactile_motor_indices[channel] == motor_index) {
            return (int8_t)channel;
        }
    }
    return -1;
}

// Motor_ConvertJointAngleTenths：按原有关节标定换算0.1度单位的二进制角度
// 参数：motor_index - 关节下标；adc_value - ADC均值；tenths - 输出地址
// 返回值：true表示端点与计算合法，false表示参数或标定无效
bool Motor_ConvertJointAngleTenths(uint8_t motor_index, uint16_t adc_value, uint16_t *tenths)
{
    if (motor_index >= Motor_Num || tenths == NULL) {
        return false;
    }
    float adc_zero = ADValue[motor_index][0];
    float delta = ADValue[motor_index][1] - adc_zero;
    if (!isfinite(adc_zero) || !isfinite(delta) || fabsf(delta) < 0.001f) {
        return false;
    }
    float angle = ((float)adc_value - adc_zero) * 900.0f / delta;
    if (!isfinite(angle)) {
        return false;
    }
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 900.0f) angle = 900.0f;
    *tenths = (uint16_t)(angle + 0.5f);
    return true;
}

// Motor_GetTelemetrySnapshot：从独立采集缓存复制CAN需要的完整软件快照
// 参数：snapshot - 输出地址
// 返回值：无；内部短临界区不依赖电机寄存器锁
void Motor_GetTelemetrySnapshot(MotorTelemetrySnapshot_t *snapshot)
{
    SensorFeedback_ReadSnapshot(snapshot);
}
