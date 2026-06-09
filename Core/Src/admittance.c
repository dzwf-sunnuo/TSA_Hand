#include "admittance.h"
#include <math.h>   // fabsf

// 触觉传感器死区 (N), |F| < 此值视为噪声, 不触发导纳响应
#define FORCE_DEADBAND  0.05f

void Admittance_Init(Admittance_Ctrl *adm, float M, float B, float K, float x_max)
{
    adm->M     = M;
    adm->B     = B;
    adm->K     = K;
    adm->x     = 0.0f;
    adm->v     = 0.0f;
    adm->x_max = x_max;
}

/**
 * @brief 导纳控制更新框架
 *
 * 物理模型:  M·Δẍ + B·Δẋ + K·Δx = F_ext
 *
 * 输入: 触觉传感器力值 F_ext (N), 正=受压, 负=受拉
 * 输出: 位置修正量 Δx (度), 调用方加到 TargetAngle
 *
 * 调用周期: 10ms (与 Motor_Control_Loop 同频)
 *
 * ┌─────────────────────────────────────────────────────┐
 * │  阶段1: 力预处理 (校准 + 死区)                       │
 * │  阶段2: 接触判断 (自由空间 / 接触中)                  │
 * │  阶段3: 导纳动力学计算 (弹簧-阻尼-质量模型)             │
 * │  阶段4: 输出限幅 + 状态更新                           │
 * └─────────────────────────────────────────────────────┘
 */
float Admittance_Update(Admittance_Ctrl *adm, float force, float dt)
{
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  阶段1: 力预处理
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    // 1.1 死区滤波: 低于门限的力视为传感器噪声, 置零
    if (fabsf(force) < FORCE_DEADBAND) {
        force = 0.0f;
    }

    // 1.2 (预留) 低通滤波: 对 force 做一阶 IIR 滤波, 消除高频毛刺
    // force_filtered = alpha * force + (1-alpha) * force_filtered_prev;

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  阶段2: 接触状态判断
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    int in_contact = (fabsf(force) > 1e-6f);  // force ≠ 0 → 接触中

    if (!in_contact && fabsf(adm->x) < 1e-6f) {
        // 情况A: 自由空间 + 无累积偏移 → 无需计算, 直接返回 0
        // 这是最常见的状态 (手指没碰到任何东西)
        adm->v = 0.0f;     // 速度也清零, 防止残留
        return 0.0f;
    }

    // 情况B: 接触中 → 力驱动导纳模型, 计算修正量
    // 情况C: 自由空间但 x≠0 → 之前接触过, 现在正在恢复, 导纳模型会使 x 自动归零
    //         (此时 force=0, 弹簧 K 会把 Δx 拉回零点)

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  阶段3: 导纳动力学计算
    //
    //  连续域:  M·Δẍ + B·Δẋ + K·Δx = F_ext
    //
    //  当 M=0 (一阶简化, 推荐嵌入式):
    //    离散公式 (前向欧拉, dt=0.01):
    //      Δx[k] = (F_ext[k] + B/dt·Δx[k-1]) / (B/dt + K)
    //
    //  参数物理意义:
    //    K: 虚拟弹簧 → 力越大退让越多, K 越小越"软"
    //    B: 虚拟阻尼 → 阻止快速运动, B 越大响应越平缓
    //    M: 虚拟惯量 → 产生"重量感", 嵌入式通常置 0
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    float x_new, v_new;

    if (adm->M < 1e-6f) {
        // ---- 一阶模型 (无惯量) ----
        float B_over_dt = adm->B / dt;
        float denom     = adm->K + B_over_dt;

        x_new = (force + B_over_dt * adm->x) / denom;
        v_new = (x_new - adm->x) / dt;

    } else {
        // ---- 二阶模型 (含惯量) ----
        float accel = (force - adm->B * adm->v - adm->K * adm->x) / adm->M;

        x_new = adm->x + dt * adm->v;
        v_new = adm->v + dt * accel;
    }

    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    //  阶段4: 输出限幅 + 状态保存
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    // 4.1 位置限幅: 防止松手过度, 手指不能退让超过 ±x_max
    if      (x_new >  adm->x_max) x_new =  adm->x_max;
    else if (x_new < -adm->x_max) x_new = -adm->x_max;

    // 4.2 状态保存, 供下个周期计算阻尼项和惯量项
    adm->x = x_new;
    adm->v = v_new;

    // 4.3 返回位置修正量, 调用方加到 TargetAngle
    return x_new;
}

void Admittance_Reset(Admittance_Ctrl *adm)
{
    adm->x = 0.0f;
    adm->v = 0.0f;
}
