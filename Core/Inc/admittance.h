#ifndef __ADMITTANCE_H__
#define __ADMITTANCE_H__

#include <stdint.h>

// 导纳控制参数结构体 (每指独立)
typedef struct {
    // --- 虚拟力学参数 ---
    float M;     // 虚拟惯量 (kg), 0=无惯量模式
    float B;     // 虚拟阻尼 (N·s/m), 越大运动越粘滞
    float K;     // 虚拟刚度 (N/m), 越小手越"软", 越大越"硬"

    // --- 内部状态 ---
    float x;     // 位置修正量 Δx (度), 当前累积值
    float v;     // 速度修正量 Δv (度/s), 当前累积值

    // --- 限幅 ---
    float x_max; // 位置修正最大幅度 (度), 防止松手过度
} Admittance_Ctrl;

// 初始化导纳控制器
// adm     - 指向控制器实例
// M       - 虚拟惯量, 设 0 使用一阶简化模型 (推荐嵌入式)
// B       - 虚拟阻尼, 典型 0.5~5.0, 越大响应越平滑
// K       - 虚拟刚度, 典型 5.0~50.0, 越小越"顺从"
// x_max   - 最大位置修正量 (度), 如 10.0 表示最多退让 10°
void Admittance_Init(Admittance_Ctrl *adm, float M, float B, float K, float x_max);

// 导纳控制更新 (每 10ms 调用一次)
// adm     - 控制器实例
// force   - 触觉传感器当前力值 (N), 正=受压力, 负=受拉力
// dt      - 控制周期 (秒), 通常 0.01
// 返回值  - 位置修正量 Δx (度), 直接加到 TargetAngle 上
float Admittance_Update(Admittance_Ctrl *adm, float force, float dt);

// 紧急复位 (手指脱离时清零导纳状态)
void Admittance_Reset(Admittance_Ctrl *adm);

#endif /* __ADMITTANCE_H__ */
