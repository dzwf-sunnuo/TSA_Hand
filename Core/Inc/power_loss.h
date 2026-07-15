#ifndef __POWER_LOSS_H__
#define __POWER_LOSS_H__

// 12V 主电掉电检测 (ADC2 IN4, 100K/10K 分压)
// V_ADC = 12V × 10K/(100K+10K) ≈ 1.09V → ADC ≈ 1352

// 初始化掉电检测 (启动 ADC2)
void PL_Init(void);

// 检测 12V 主电是否在线, 返回: 1=正常, 0=掉电
int PL_Is12VAlive(void);

// 紧急停机: 刹停 4 路电机 + 关 LED (PE2)
void PL_EmergencyStop(void);

// 恢复运行: 开 LED (PE2)
void PL_Resume(void);

#endif /* __POWER_LOSS_H__ */
