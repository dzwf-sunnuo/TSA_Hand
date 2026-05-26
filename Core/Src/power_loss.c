#include "power_loss.h"
#include "Motor.h"
#include "rs485.h"
#include "rs485_crc.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// 记录在 Flash 中的格式 (20 字节)
typedef struct __attribute__((packed)) {
    uint16_t magic;     // 0x5AA5
    uint16_t crc;       // CRC16 校验 (覆盖 angles[4])
    float    angles[4]; // 4 路电机出轴角度
} PL_Record_t;

static uint32_t pl_write_offset;  // 下次写入的扇区内偏移

// ================================================================
//  Flash 辅助函数
// ================================================================

// 计算 angles[4] 的 CRC16
static uint16_t pl_calc_crc(const float *angles)
{
    return Modbus_CRC16((uint8_t *)angles, 4 * sizeof(float));
}

// 从扇区扫描: 找到最后一条有效的记录, 返回其偏移 (0 表示无效)
static uint32_t pl_find_last_record(void)
{
    uint32_t last_valid = 0;
    PL_Record_t rec;

    for (uint32_t off = 0; off < PL_FLASH_SIZE; off += PL_RECORD_SIZE) {
        // 跳过未写入区 (全 0xFF)
        uint16_t *magic_ptr = (uint16_t *)(PL_FLASH_ADDR + off);
        if (*magic_ptr == 0xFFFF) continue;

        // 读取记录
        memcpy(&rec, (void *)(PL_FLASH_ADDR + off), sizeof(PL_Record_t));

        if (rec.magic != PL_MAGIC) continue;

        uint16_t calc_crc = pl_calc_crc(rec.angles);
        if (calc_crc == rec.crc) {
            last_valid = off;
        }
    }
    return last_valid;
}

// 擦除备份扇区 (仅在扇区写满时调用, 发生在 main 初始化阶段, 时间充裕)
static void pl_erase_sector(void)
{
    __disable_irq();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef erase_cfg = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = PL_FLASH_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_err = 0;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase_cfg, &sector_err);
    HAL_FLASH_Lock();
    __enable_irq();

    pl_write_offset = 0;
}

// 使指定偏移处的记录失效 (清除 magic 字段, Flash 半字写入 ~30μs)
static void pl_invalidate_record(uint32_t offset)
{
    __disable_irq();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PL_FLASH_ADDR + offset, 0x0000);
    HAL_FLASH_Lock();
    __enable_irq();
}

// ================================================================
//  上电初始化
// ================================================================
void PL_Init(float angles_out[4])
{
    // 配置 PVD: 2.9V 下降沿触发中断 (VDD 从 3.3V 跌落时触发)
    PWR_PVDTypeDef sConfigPVD = {0};
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_5;          // 阈值 2.9V
    sConfigPVD.Mode    = PWR_PVD_MODE_IT_FALLING;   // 仅下降沿触发
    HAL_PWR_ConfigPVD(&sConfigPVD);
    HAL_PWR_EnablePVD();

    // 默认值
    memset(angles_out, 0, 4 * sizeof(float));

    uint32_t off = pl_find_last_record();
    if (off >= PL_FLASH_SIZE) {
        // 无有效记录, 从头开始写 (扇区为出厂全 FF 或已全部失效)
        pl_write_offset = 0;
        return;
    }

    // 读取最后一条有效记录
    PL_Record_t rec;
    memcpy(&rec, (void *)(PL_FLASH_ADDR + off), sizeof(PL_Record_t));

    if (rec.magic == PL_MAGIC && pl_calc_crc(rec.angles) == rec.crc) {
        memcpy(angles_out, rec.angles, 4 * sizeof(float));
    }

    // 使已恢复的记录失效 (清除 magic, ~30μs), 避免下次启动重复恢复
    pl_invalidate_record(off);

    // 下次 PVD 写入位置 = 最后记录之后
    pl_write_offset = off + PL_RECORD_SIZE;
    if (pl_write_offset >= PL_FLASH_SIZE) {
        // 扇区写满时才擦除 (6553+ 次掉电才会触发, 极其罕见)
        pl_erase_sector();
    }
}

// ================================================================
//  紧急停机 (PVD 回调中调用)
// ================================================================
void PL_EmergencyStop(void)
{
    for (int i = 0; i < Motor_Num; i++) {
        Set_Motor(i, 0, 0);
    }
}

// ================================================================
//  掉电保存 (PVD 回调中调用)
// ================================================================
static void pl_save_to_flash(void)
{
    if (pl_write_offset + PL_RECORD_SIZE > PL_FLASH_SIZE) {
        return;  // 扇区写满 (极端情况), 放弃本次保存
    }

    PL_Record_t rec;
    rec.magic = PL_MAGIC;
    for (int i = 0; i < Motor_Num; i++) {
        rec.angles[i] = motor[i].CurrentAngle;
    }
    rec.crc = pl_calc_crc(rec.angles);

    uint32_t addr = PL_FLASH_ADDR + pl_write_offset;
    uint32_t *p = (uint32_t *)&rec;
    uint32_t word_count = (sizeof(PL_Record_t) + 3) / 4;

    // 关中断防止高优先级 ISR 在 Flash 写入期间访问 Flash 导致总线冲突
    __disable_irq();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < word_count; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, p[i]);
    }
    HAL_FLASH_Lock();
    __enable_irq();

    pl_write_offset += PL_RECORD_SIZE;
}

// ================================================================
//  掉电过程中不能等待, PVD 中断回调
// ================================================================
void HAL_PWR_PVDCallback(void)
{
    PL_EmergencyStop();   // 先刹停所有电机
    pl_save_to_flash();   // 写 Flash (关中断内, ~150μs)
}

// ================================================================
//  上电自主回零
//  在 main.c 调用 PL_Init 恢复角度后,
//  设置 mode=1 和 Reg=0 即可让 PID 把电机带回零点
// ================================================================
