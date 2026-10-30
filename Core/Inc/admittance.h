#ifndef __ADMITTANCE_H__
#define __ADMITTANCE_H__

#include <stdint.h>

// 连续力死区，单位牛顿；接触判断与导纳模型共用此门限，防止两处定义不一致。
#define ADMITTANCE_FORCE_DEADBAND_N 0.3f

// Admittance_Ctrl：保存每指独立的虚拟力学参数、退让状态和限幅
// 参数：无；下文以q表示退让坐标，模式6单位为圈，模式7单位为度
// 返回值：无
typedef struct {
    // --- 虚拟力学参数 ---
    float M;     // 虚拟惯量，单位牛顿·秒平方/q；零表示一阶无惯量模型
    float B;     // 虚拟阻尼，单位牛顿·秒/q；越大响应越平缓
    float K;     // 虚拟刚度，单位牛顿/q；越小则同等有效力产生的退让越大

    // --- 内部状态 ---
    float x;     // 当前累积退让量，单位q
    float v;     // 当前退让速度，单位q/秒

    // --- 限幅 ---
    float x_max; // 最大退让幅度，单位q，防止过度退让
} Admittance_Ctrl;

// Admittance_Init：初始化虚拟力学参数并清零退让状态
// 参数：adm - 控制器实例；M - 虚拟惯量；B - 虚拟阻尼；K - 虚拟刚度；x_max - 最大退让量
// 返回值：无
// adm     - 指向控制器实例
// M       - 虚拟惯量, 设 0 使用一阶简化模型 (推荐嵌入式)
// B       - 虚拟阻尼，当前上电默认0.025
// K       - 虚拟刚度，当前接触默认0.05，回弹由调用方切换为0.2
// x_max   - 最大退让量，单位随模式使用输出轴圈数或关节角度
void Admittance_Init(Admittance_Ctrl *adm, float M, float B, float K, float x_max);

// Admittance_Update：对原始力执行一次连续死区处理，并推进导纳模型
// 参数：adm - 控制器实例；force - 未扣除死区的原始力，单位牛顿；dt - 控制周期，单位秒
// 返回值：退让量；调用方从基准目标中减去该值，禁止在调用前再次扣除死区
// adm     - 控制器实例
// force   - 触觉传感器当前力值 (N), 正=受压力, 负=受拉力
// dt      - 控制周期 (秒), 通常 0.01
// 退让量单位随模式使用输出轴圈数或关节角度，正值表示减小目标。
float Admittance_Update(Admittance_Ctrl *adm, float force, float dt);

// Admittance_Reset：清零累积退让量及速度，保留导纳参数
// 参数：adm - 控制器实例
// 返回值：无
void Admittance_Reset(Admittance_Ctrl *adm);

#endif /* __ADMITTANCE_H__ */
