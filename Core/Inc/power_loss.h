#ifndef __POWER_LOSS_H__
#define __POWER_LOSS_H__

#include <stdint.h>

// 掉电保存结构体: 4 个电机的出轴角度 (float) + CRC + magic
#define PL_RECORD_SIZE      20   // 字节
#define PL_MAGIC            0x5AA5

// 备份扇区: STM32F407VETx Sector 5 (0x08020000, 128KB)
#define PL_FLASH_SECTOR     FLASH_SECTOR_5
#define PL_FLASH_ADDR       0x08020000
#define PL_FLASH_SIZE       0x20000   // 128KB
#define PL_MAX_RECORDS      (PL_FLASH_SIZE / PL_RECORD_SIZE)  // ~6553 条

// 初始化: 上电时调用, 扫描扇区恢复最近记录, 然后擦除扇区
// 返回恢复的电机角度 (4 个 float), 如果无有效记录则全为 0
void PL_Init(float angles_out[4]);

// 紧急停电机 (PVD 回调中调用, 把所有 PWM 清零)
void PL_EmergencyStop(void);

#endif /* __POWER_LOSS_H__ */
