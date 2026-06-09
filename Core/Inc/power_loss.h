#ifndef __POWER_LOSS_H__
#define __POWER_LOSS_H__

#include <stdint.h>

// 备份寄存器双槽布局 (每槽 8 个 32 位寄存器, STM32F407 共 20 个 BKP)
// 槽A: BKP0~BKP7    槽B: BKP8~BKP15
// 槽内: [magic][seq][angle0][angle1][angle2][angle3][ver+CRC][commit]
#define PL_SLOT_A_BASE   0     // BKP0R
#define PL_SLOT_B_BASE   8     // BKP8R
#define PL_SLOT_REGS     8     // 每槽寄存器数

// 槽内字偏移
#define PL_OFF_MAGIC     0
#define PL_OFF_SEQ       1
#define PL_OFF_ANGLE0    2     // angle[0~3] 占 offset 2~5
#define PL_OFF_CRC       6     // [31:16]版本号 [15:0]CRC16 (覆盖 magic+seq+4 angle)
#define PL_OFF_COMMIT    7     // 最后写入, 保证原子性

#define PL_MAGIC         0x504C5331  // "PLS1"
#define PL_COMMIT        0x00000001
#define PL_COMMIT_INVALID 0xFFFFFFFF  // 写入前标记为无效，写入成功后再写 PL_COMMIT
#define PL_VERSION       0x0001

// 初始化: 上电时调用, 扫描两槽恢复最近一次有效快照
// angles_out: 输出的 4 个 float 角度, 无有效记录则全为 0
void PL_Init(float angles_out[4]);

// 保存当前电机角度到备份寄存器 (在 10ms 电机控制循环末尾调用)
// 双槽交替写入, 掉电最多丢失一帧 (10ms)
void PL_SaveAngles(void);

// 紧急停机 (PVD 回调中调用, 把 4 路 PWM 清零)
void PL_EmergencyStop(void);

#endif /* __POWER_LOSS_H__ */
