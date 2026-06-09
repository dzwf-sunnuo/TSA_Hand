#include "power_loss.h"
#include "Motor.h"
#include "rs485_crc.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// 备份寄存器指针基址 (BKP0R~BKP19R 在 RTC 外设内, 地址连续)
static __IO uint32_t *const bkp = &RTC->BKP0R;

static uint32_t pl_seq = 0;   // 递增序号
static int pl_toggle = 0;     // 0→下一帧写槽A, 1→下一帧写槽B

// ================================================================
//  备份寄存器读写辅助
// ================================================================

// seq 回绕安全比较: (int32_t)(a - b) > 0 ⇔ a 比 b 新
static inline int pl_seq_gt(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}

// CRC 覆盖 6 个 32 位字: magic + seq + angle[0..3]
static uint16_t pl_calc_slot_crc(const uint32_t data[6])
{
    return Modbus_CRC16((const uint8_t *)data, 6 * sizeof(uint32_t));
}

// ================================================================
//  双槽读写
// ================================================================

// 读一槽: 校验 commit → magic → CRC, 通过后输出 angles
// 返回 0=有效, -1=无效
static int pl_read_slot(uint32_t base, float angles_out[4])
{
    // commit 是最后一个写入的字, 如果缺失说明掉电时未写完
    if (bkp[base + PL_OFF_COMMIT] != PL_COMMIT) return -1;
    if (bkp[base + PL_OFF_MAGIC]  != PL_MAGIC)   return -1;

    // 组装 CRC 输入: magic + seq + 4×angle
    uint32_t crc_in[6];
    for (int i = 0; i < 6; i++) {
        crc_in[i] = bkp[base + i];
    }
    uint16_t calc = pl_calc_slot_crc(crc_in);
    uint16_t stored = (uint16_t)(bkp[base + PL_OFF_CRC] & 0xFFFF);
    if (calc != stored) return -1;

    // 恢复角度 (float 原始位 → float)
    for (int i = 0; i < 4; i++) {
        uint32_t bits = bkp[base + PL_OFF_ANGLE0 + i];
        memcpy(&angles_out[i], &bits, sizeof(float));
    }
    return 0;
}

// 写一槽: 按顺序写入, commit 最后 (保证原子性)
static void pl_write_slot(uint32_t base, uint32_t seq, const float angles[4])
{
    uint32_t crc_in[6];

    crc_in[0] = PL_MAGIC;
    crc_in[1] = seq;
    for (int i = 0; i < 4; i++) {
        memcpy(&crc_in[2 + i], &angles[i], sizeof(float));
    }
    uint16_t crc = pl_calc_slot_crc(crc_in);

    // 使能备份域写权限
    HAL_PWR_EnableBkUpAccess();

    // 写入前先把 commit 标记为无效，防止写到一半被判为已提交
    bkp[base + PL_OFF_COMMIT]  = PL_COMMIT_INVALID;

    // 顺序写入, commit 必须最后
    bkp[base + PL_OFF_MAGIC]   = PL_MAGIC;
    bkp[base + PL_OFF_SEQ]     = seq;
    bkp[base + PL_OFF_ANGLE0]  = crc_in[2];
    bkp[base + PL_OFF_ANGLE0+1]= crc_in[3];
    bkp[base + PL_OFF_ANGLE0+2]= crc_in[4];
    bkp[base + PL_OFF_ANGLE0+3]= crc_in[5];
    bkp[base + PL_OFF_CRC]     = ((uint32_t)PL_VERSION << 16) | crc;
    // 最后写入 commit, 掉电发生在此之前 → 该槽校验失败, 对侧槽仍然有效
    bkp[base + PL_OFF_COMMIT]  = PL_COMMIT;
}

// ================================================================
//  上电初始化
// ================================================================
void PL_Init(float angles_out[4])
{
    // 配置 PVD: 2.9V 下降沿触发中断 (掉电时紧急刹停)
    PWR_PVDTypeDef sConfigPVD = {0};
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_7;          // 阈值 2.9V
    sConfigPVD.Mode    = PWR_PVD_MODE_IT_FALLING;   // 仅下降沿触发
    HAL_PWR_ConfigPVD(&sConfigPVD);
    HAL_PWR_EnablePVD();

    memset(angles_out, 0, 4 * sizeof(float));

    float slot_a[4], slot_b[4];
    int va = pl_read_slot(PL_SLOT_A_BASE, slot_a);
    int vb = pl_read_slot(PL_SLOT_B_BASE, slot_b);

    if (va == 0 && vb == 0) {
        // 两槽均有效 → 比 seq 选更新的
        uint32_t seq_a = bkp[PL_SLOT_A_BASE + PL_OFF_SEQ];
        uint32_t seq_b = bkp[PL_SLOT_B_BASE + PL_OFF_SEQ];
        if (pl_seq_gt(seq_a, seq_b)) {
            memcpy(angles_out, slot_a, sizeof(slot_a));
            pl_seq = seq_a + 1;
        } else {
            memcpy(angles_out, slot_b, sizeof(slot_b));
            pl_seq = seq_b + 1;
        }
        // 下一帧写到较旧的那个槽 (如果 A 新则 A 刚被读过, 写 B)
        pl_toggle = pl_seq_gt(seq_a, seq_b) ? 1 : 0;
    } else if (va == 0) {
        memcpy(angles_out, slot_a, sizeof(slot_a));
        pl_seq = bkp[PL_SLOT_A_BASE + PL_OFF_SEQ] + 1;
        pl_toggle = 1;  // 槽 A 有数据, 下一帧写槽 B
    } else if (vb == 0) {
        memcpy(angles_out, slot_b, sizeof(slot_b));
        pl_seq = bkp[PL_SLOT_B_BASE + PL_OFF_SEQ] + 1;
        pl_toggle = 0;  // 槽 B 有数据, 下一帧写槽 A
    } else {
        // 两槽都无效 (首次上电 / VBAT 掉过)
        pl_seq = 0;
        pl_toggle = 0;
    }
}

// ================================================================
//  周期保存 (在 10ms 电机控制任务末尾调用)
// ================================================================
void PL_SaveAngles(void)
{
    float angles[4];
    for (int i = 0; i < Motor_Num; i++) {
        angles[i] = motor[i].CurrentAngle;
    }

    uint32_t base = pl_toggle ? PL_SLOT_B_BASE : PL_SLOT_A_BASE;
    pl_write_slot(base, pl_seq, angles);

    pl_seq++;
    pl_toggle = !pl_toggle;
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
//  PVD 中断回调 (最小化: 仅刹停, 不写 Flash)
//  真正的数据持久化由 10ms 任务的 PL_SaveAngles 完成
// ================================================================
void HAL_PWR_PVDCallback(void)
{
    PL_EmergencyStop();
}
