#include "Motor.h"
#include "stdio.h"
#include "adc.h"
#include "usart.h"
#include "rs485.h"
#include "rs485_crc.h"

/* 堵转保护 */
#define STALL_TIMEOUT_SECONDS 3.0f   // 堵转判定时间 (秒)
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

uint16_t ADC_HallValue[Hall_Num] __CCM_RAM_DATA = {0}; // 经过滤波后的 ADC 原始值

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
    {1560.0f, 2580.0f}, // 关节 1
    {2475.0f, 1435.0f}, // 关节 2
    {2500.0f, 1450.0f}, // 关节 3
    {2565.0f, 1540.0f}  // 关节 4
} ;

__CCM_RAM_DATA static int InterpolationFlag[4] = {11, 11, 11, 11} ; // 插值搜索方向标志

/* PID 初始参数 */
__CCM_RAM_DATA float Kp_Position[Hall_Num]  = {5.0f, 8.0f, 8.0f, 8.0f} ;
__CCM_RAM_DATA float Ki_Position[Hall_Num] = {0.0f, 0.0f, 0.0f, 0.0f} ;
__CCM_RAM_DATA float Kd_Position[Hall_Num] = {0.0f, 0.0f, 0.0f, 0.0f} ;

__CCM_RAM_DATA float Kp_Angle[Motor_Num] = {0.5f, 0.7f, 0.7f, 0.7f} ;
__CCM_RAM_DATA float Ki_Angle[Motor_Num] = {0.0f, 0.0f, 0.0f, 0.0f} ;
__CCM_RAM_DATA float Kd_Angle[Motor_Num] = {0.0f, 0.0f, 0.0f, 0.0f} ;

__CCM_RAM_DATA float Kp_Speed[Motor_Num] = {0.3f, 0.3f, 0.3f, 0.3f} ;
__CCM_RAM_DATA float Ki_Speed[Motor_Num] = {0.1f, 0.1f, 0.1f, 0.1f} ;
__CCM_RAM_DATA float Kd_Speed[Motor_Num] = {0.0f, 0.0f, 0.0f, 0.0f} ;

/**
 * @brief 电机控制系统初始化
 * 
 * 开启所有电机的 PWM 输出、编码器计数，并初始化 PID 结构体参数。
 */
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
    /* 200ms 步进的后台计时逻辑 (例如 1s 定时发送) */
    modbus.Host_Sendtime++;
    if (modbus.Host_Sendtime >= 5) { 
        modbus.Host_time_flag = 1; // 1秒定时到达标志
        modbus.Host_Sendtime = 0;  // 自动复位计数器

        /*某些东西。。。*/


    }
}

__CCM_RAM_TEXT void Motor_Control_Loop(void)
{
    /* [2] 10ms 电机控制闭环逻辑 */
        uint8_t mode = Reg[4] >> 8;     // 从 Modbus 寄存器 Reg[4] 高 8 位获取运动模式
        uint8_t io_flag = Reg[4] & 0xFF; // 从 Modbus 寄存器 Reg[4] 低 8 位获取启停标志

        /* 堵转保护状态 (跨调用保持) */
        static uint16_t stall_timer[Motor_Num] = {0};
        static float    stall_target[Motor_Num] = {0};

        for (int i = 0; i < Motor_Num; i++) {
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

            /* ---- 堵转保护 ---- */
            if (io_flag && mode != 4) {
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
                    if (motor[i].TargetAngle > Real_OneTurn * 40.0f) motor[i].TargetAngle = Real_OneTurn * 40.0f;
                    if (motor[i].TargetAngle < -Real_OneTurn * 40.0f) motor[i].TargetAngle = -Real_OneTurn * 40.0f;

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
                case 5: // 模式5：关节角度插值控制 (接收 Reg[i] 为目标关节物理角度)
                {
                    float target_joint_angle = (float)Reg[i];
                    if (target_joint_angle < 5.0f) target_joint_angle = 5.0f;   // 范围限制 5~90度
                    if (target_joint_angle > 90.0f) target_joint_angle = 90.0f;

                    // 通过线性插值将目标物理角度转换为目标 ADC 值
                    motor[i].TargetPosition = lineInterp(JointAngle[i], ADValue[i], CalibrationLEN, target_joint_angle, InterpolationFlag[i]);
                    // 位置环计算 -> 得到目标速度
                    motor[i].TargetSpeed = -PID_Increment(&PID_Position[i], motor[i].CurrentPosition, motor[i].TargetPosition);

                    // 死区处理：ADC 误差在 5 以内则停止
                    if (PID_Position[i].Error_Last1 <= 5.0f && PID_Position[i].Error_Last1 >= -5.0f) motor[i].TargetSpeed = 0.0f;
                    // 安全保护：若电机转动超过 40 圈则强制停止以防拉断
                    if (motor[i].CurrentAngle < -40.0f * Real_OneTurn || motor[i].CurrentAngle > 40.0f * Real_OneTurn) motor[i].TargetSpeed = 0.0f;
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
__CCM_RAM_TEXT void Set_Motor(uint8_t i, int16_t set_speed, uint8_t flag)
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
__CCM_RAM_TEXT float PID_Increment(PID_Increment_Struct *PID, float Current, float Target)
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

//打印特定电机的目标出轴角度，当前出轴角度，目标速度，当前速度
//（两个参数，第一个参数是电机索引，0-3，第二个是要看的参数，1对应目标出轴速度和当前出轴速度，2对应目标出轴角度和当前出轴角度）
void Print_Motor_Status(uint8_t motor_index, uint8_t param)
{
    if (motor_index >= Motor_Num) return;
    if (param == 1) {// 打印速度信息
        printf("Motor %d Target Speed: %.2f RPM, Actual Speed: %.2f RPM\r\n", motor_index + 1, motor[motor_index].TargetSpeed, motor[motor_index].CurrentSpeed);
    } else if (param == 2) {// 打印角度信息
        printf("Motor %d Target Angle: %.2f deg, Actual Angle: %.2f deg\r\n", motor_index + 1, motor[motor_index].TargetAngle , motor[motor_index].CurrentAngle);
    }
}//用于调试，定期打印电机状态信息，观察 PID 收敛情况和系统响应特性
