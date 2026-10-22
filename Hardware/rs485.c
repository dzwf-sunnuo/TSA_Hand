#include "rs485.h"
#include "usart.h"
#include "wrist.h"
#include "app_comm_tasks.h"
#include "uart4_telemetry.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <string.h>

extern volatile uint16_t cycle_time;

MODBUS modbus1; // 历史名称保留，实际通过USART2控制手指从板。
MODBUS modbus2; // 历史名称保留，实际通过USART1响应上位机。
uint16_t Reg[MODBUS_REG_COUNT] = {0};
static osMutexId_t s_master_tx_mutex;

// Modbus_ReadU16：读取协议中的两字节大端整数
// 参数：data - 两字节起始地址
// 返回值：无符号16位数值
static uint16_t Modbus_ReadU16(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

// Modbus_SendResponse：追加沿用项目CRC编码的校验值并整帧回复
// 参数：length - 不含CRC的响应长度
// 返回值：无，广播写入不产生响应
static void Modbus_SendResponse(uint16_t length)
{
    if (modbus2.rcbuf[0] == 0U || length + 2U > sizeof(modbus2.sendbuf)) return;
    uint16_t crc = Modbus_CRC16(modbus2.sendbuf, length);
    modbus2.sendbuf[length++] = (uint8_t)(crc >> 8);
    modbus2.sendbuf[length++] = (uint8_t)crc;
    (void)HAL_UART_Transmit(&huart1, modbus2.sendbuf, length, 20U);
}

// Modbus_SendException：回复功能码异常、地址异常或数据异常
// 参数：code - MODBUS异常码
// 返回值：无，广播帧不回复
static void Modbus_SendException(uint8_t code)
{
    modbus2.sendbuf[0] = modbus2.myadd;
    modbus2.sendbuf[1] = modbus2.rcbuf[1] | 0x80U;
    modbus2.sendbuf[2] = code;
    Modbus_SendResponse(3U);
}

// Modbus_RangeOverlaps：判断写入区间是否触及某一业务寄存器区间
// 参数：start、count - 写入范围；first、last - 闭区间业务范围
// 返回值：true表示有交集
static bool Modbus_RangeOverlaps(uint16_t start, uint16_t count,
                                 uint16_t first, uint16_t last)
{
    return start <= last && (uint32_t)start + count > first;
}

// Modbus_ValidateValue：校验新增上报寄存器的可写值
// 参数：address - 寄存器地址；value - 候选值
// 返回值：true表示允许写入，两个阈值允许完整16位范围
static bool Modbus_ValidateValue(uint16_t address, uint16_t value)
{
    if (address == UART4_REPORT_REG_ENABLE) return value <= 1U;
    if (address == UART4_REPORT_REG_FREQUENCY) {
        return value >= 1U && value <= UART4_REPORT_MAX_FREQUENCY;
    }
    return true;
}

// Modbus_ApplyWrite：按已提交的地址范围触发控制或上报配置通知
// 参数：start - 写入首地址；count - 已成功写入的寄存器数
// 返回值：无，配置写入及读取不会重复下发任何电机动作
static void Modbus_ApplyWrite(uint16_t start, uint16_t count)
{
    uint8_t boards = 0U;
    if (Modbus_RangeOverlaps(start, count, 4U, 8U)) boards |= 0x01U;
    if (Modbus_RangeOverlaps(start, count, 9U, 13U)) boards |= 0x02U;
    if (boards != 0U) App_RequestSlaveDispatch(boards);

    bool control_changed = Modbus_RangeOverlaps(start, count, 14U, 14U);
    if ((Reg[14] & 0xFFU) == 1U) {
        if (control_changed || Modbus_RangeOverlaps(start, count, 0U, 1U)) {
            vel_move((uint8_t)(Reg[0] >> 8), (uint8_t)Reg[0],
                     (uint8_t)(Reg[1] >> 8), (uint8_t)Reg[1]);
        }
        if (control_changed || Modbus_RangeOverlaps(start, count, 2U, 3U)) {
            vel_move1((uint8_t)(Reg[2] >> 8), (uint8_t)Reg[2],
                      (uint8_t)(Reg[3] >> 8), (uint8_t)Reg[3]);
        }
    }
    if (control_changed || Modbus_RangeOverlaps(start, count, 15U, 15U)) {
        cycle_time = Reg[15];
        if ((Reg[14] >> 8) != 0U) WristMotion_Start(MODE_PENDULUM);
        else WristMotion_Stop();
    }
    if (Modbus_RangeOverlaps(start, count, UART4_REPORT_REG_ENABLE,
                             UART4_REPORT_REG_FORCE_MIN)) {
        Uart4Telemetry_NotifyConfigChanged();
    }
}

// modbus2_Send_Byte：向USART1上位机接口发送单字节
// 参数：ch - 待发送字节
// 返回值：无
void modbus2_Send_Byte(uint8_t ch)
{
    (void)HAL_UART_Transmit(&huart1, &ch, 1U, 20U);
}

// modbus1_Send_Byte：通过USART2发送单字节，串行化主机发送
// 参数：ch - 待发送字节
// 返回值：无
void modbus1_Send_Byte(uint8_t ch)
{
    if (s_master_tx_mutex == NULL ||
        osMutexAcquire(s_master_tx_mutex, osWaitForever) != osOK) return;
    (void)HAL_UART_Transmit(&huart2, &ch, 1U, 20U);
    (void)osMutexRelease(s_master_tx_mutex);
}

// modbus2_Init：设置主板地址、原控制默认值和UART4上报默认值
// 参数：无，在调度器启动前调用
// 返回值：无
void modbus2_Init(void)
{
    memset(&modbus2, 0, sizeof(modbus2));
    memset(&modbus1, 0, sizeof(modbus1));
    memset(Reg, 0, sizeof(Reg));
    Reg[14] = 0x0001U;
    Reg[15] = 0x0898U; // 保留原来的2200毫秒手腕摆动周期。
    Reg[16] = 0x012CU;
    Reg[17] = 0x06A4U;
    Reg[UART4_REPORT_REG_ENABLE] = UART4_REPORT_DEFAULT_ENABLE;
    Reg[UART4_REPORT_REG_FREQUENCY] = UART4_REPORT_DEFAULT_FREQUENCY;
    Reg[UART4_REPORT_REG_RATIO] = UART4_REPORT_DEFAULT_RATIO;
    Reg[UART4_REPORT_REG_FORCE_MIN] = UART4_REPORT_DEFAULT_FORCE_MIN;
    modbus2.myadd = 0x10U;
}

// modbus2_Event：校验帧边界、地址和CRC，再交由对应功能码处理
// 参数：无，读取DMA完成后交付的接收缓冲区
// 返回值：无；错误帧和读取请求不会触发控制分发
void modbus2_Event(void)
{
    if (modbus2.reflag == 0U) return;
    uint16_t length = modbus2.recount;
    if (length >= 4U && length <= sizeof(modbus2.rcbuf)) {
        uint16_t received_crc = Modbus_ReadU16(&modbus2.rcbuf[length - 2U]);
        if ((modbus2.rcbuf[0] == modbus2.myadd || modbus2.rcbuf[0] == 0U) &&
            Modbus_CRC16(modbus2.rcbuf, length - 2U) == received_crc) {
            switch (modbus2.rcbuf[1]) {
            case 0x03U:
                if (modbus2.rcbuf[0] != 0U) modbus2_Func3();
                break;
            case 0x06U: modbus2_Func6(); break;
            case 0x10U: modbus2_Func16(); break;
            default: Modbus_SendException(0x01U); break;
            }
        }
    }
    modbus2.recount = 0U;
    modbus2.reflag = 0U;
}

// modbus2_Func3：读取保持寄存器并限制响应缓冲区长度
// 参数：无，请求必须恰好为8字节
// 返回值：无；非法数量返回数据异常，越界返回地址异常
void modbus2_Func3(void)
{
    if (modbus2.recount != 8U) { Modbus_SendException(0x03U); return; }
    uint16_t start = Modbus_ReadU16(&modbus2.rcbuf[2]);
    uint16_t count = Modbus_ReadU16(&modbus2.rcbuf[4]);
    if (count == 0U || 5U + 2U * (uint32_t)count > sizeof(modbus2.sendbuf)) {
        Modbus_SendException(0x03U); return;
    }
    if ((uint32_t)start + count > MODBUS_REG_COUNT) {
        Modbus_SendException(0x02U); return;
    }
    modbus2.sendbuf[0] = modbus2.myadd;
    modbus2.sendbuf[1] = 0x03U;
    modbus2.sendbuf[2] = (uint8_t)(count * 2U);
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < count; ++i) {
        modbus2.sendbuf[3U + 2U * i] = (uint8_t)(Reg[start + i] >> 8);
        modbus2.sendbuf[4U + 2U * i] = (uint8_t)Reg[start + i];
    }
    taskEXIT_CRITICAL();
    Modbus_SendResponse(3U + 2U * count);
}

// modbus2_Func6：校验单个写入值，提交后只触发命中的业务
// 参数：无，请求必须恰好为8字节
// 返回值：无
void modbus2_Func6(void)
{
    if (modbus2.recount != 8U) { Modbus_SendException(0x03U); return; }
    uint16_t start = Modbus_ReadU16(&modbus2.rcbuf[2]);
    uint16_t value = Modbus_ReadU16(&modbus2.rcbuf[4]);
    if (start >= MODBUS_REG_COUNT) { Modbus_SendException(0x02U); return; }
    if (!Modbus_ValidateValue(start, value)) { Modbus_SendException(0x03U); return; }
    taskENTER_CRITICAL();
    Reg[start] = value;
    taskEXIT_CRITICAL();
    memcpy(modbus2.sendbuf, modbus2.rcbuf, 6U);
    Modbus_SendResponse(6U);
    Modbus_ApplyWrite(start, 1U);
}

// modbus2_Func16：预校验整帧全部数值后一次性提交，拒绝部分成功
// 参数：无，请求长度须等于9加两倍寄存器数
// 返回值：无
void modbus2_Func16(void)
{
    if (modbus2.recount < 9U) { Modbus_SendException(0x03U); return; }
    uint16_t start = Modbus_ReadU16(&modbus2.rcbuf[2]);
    uint16_t count = Modbus_ReadU16(&modbus2.rcbuf[4]);
    uint32_t bytes = 2U * (uint32_t)count;
    if (count == 0U || bytes + 9U != modbus2.recount ||
        bytes != modbus2.rcbuf[6]) { Modbus_SendException(0x03U); return; }
    if ((uint32_t)start + count > MODBUS_REG_COUNT) {
        Modbus_SendException(0x02U); return;
    }
    for (uint16_t i = 0U; i < count; ++i) {
        if (!Modbus_ValidateValue(start + i, Modbus_ReadU16(&modbus2.rcbuf[7U + 2U * i]))) {
            Modbus_SendException(0x03U); return;
        }
    }
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < count; ++i) {
        Reg[start + i] = Modbus_ReadU16(&modbus2.rcbuf[7U + 2U * i]);
    }
    taskEXIT_CRITICAL();
    memcpy(modbus2.sendbuf, modbus2.rcbuf, 6U);
    Modbus_SendResponse(6U);
    Modbus_ApplyWrite(start, count);
}

// ModbusMaster_Init：创建带优先级继承的USART2发送互斥锁
// 参数：无，内核初始化后调用
// 返回值：非0表示成功
uint8_t ModbusMaster_Init(void)
{
    const osMutexAttr_t attr = {.name = "SlaveBusTx", .attr_bits = osMutexPrioInherit};
    s_master_tx_mutex = osMutexNew(&attr);
    return s_master_tx_mutex != NULL;
}

// Host_write06_slave：互斥使用主机缓冲区，通过USART2写一个寄存器
// 参数：slave - 从板地址；fun - 功能码；StartAddr - 地址；num - 数值
// 返回值：无，Host_send_flag记录本次发送是否成功
void Host_write06_slave(uint8_t slave, uint8_t fun, uint16_t StartAddr, uint16_t num)
{
    if (s_master_tx_mutex == NULL ||
        osMutexAcquire(s_master_tx_mutex, osWaitForever) != osOK) return;
    modbus1.slave_add = slave;
    modbus1.Host_Txbuf[0] = slave;
    modbus1.Host_Txbuf[1] = fun;
    modbus1.Host_Txbuf[2] = (uint8_t)(StartAddr >> 8);
    modbus1.Host_Txbuf[3] = (uint8_t)StartAddr;
    modbus1.Host_Txbuf[4] = (uint8_t)(num >> 8);
    modbus1.Host_Txbuf[5] = (uint8_t)num;
    uint16_t crc = Modbus_CRC16(modbus1.Host_Txbuf, 6U);
    modbus1.Host_Txbuf[6] = (uint8_t)(crc >> 8);
    modbus1.Host_Txbuf[7] = (uint8_t)crc;
    modbus1.Host_send_flag = HAL_UART_Transmit(&huart2, modbus1.Host_Txbuf, 8U, 100U) == HAL_OK;
    (void)osMutexRelease(s_master_tx_mutex);
}

// Host_write16_slave：检查主机缓冲区容量后互斥发送多个寄存器
// 参数：slave - 从板地址；fun - 功能码；StartAddr - 起始地址；REnum - 数量；BYTEnum - 数据字节数；ModbusData - 数据区
// 返回值：无，Host_send_flag记录本次发送是否成功
void Host_write16_slave(uint8_t slave, uint8_t fun, uint16_t StartAddr,
                        uint16_t REnum, uint8_t BYTEnum, uint16_t *ModbusData)
{
    if (ModbusData == NULL || REnum == 0U ||
        9U + 2U * (uint32_t)REnum > sizeof(modbus1.Host_Txbuf) ||
        BYTEnum != 2U * REnum) return;
    if (s_master_tx_mutex == NULL ||
        osMutexAcquire(s_master_tx_mutex, osWaitForever) != osOK) return;
    uint16_t length = 0U;
    modbus1.slave_add = slave;
    modbus1.Host_Txbuf[length++] = slave;
    modbus1.Host_Txbuf[length++] = fun;
    modbus1.Host_Txbuf[length++] = (uint8_t)(StartAddr >> 8);
    modbus1.Host_Txbuf[length++] = (uint8_t)StartAddr;
    modbus1.Host_Txbuf[length++] = (uint8_t)(REnum >> 8);
    modbus1.Host_Txbuf[length++] = (uint8_t)REnum;
    modbus1.Host_Txbuf[length++] = BYTEnum;
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < REnum; ++i) {
        modbus1.Host_Txbuf[length++] = (uint8_t)(ModbusData[i] >> 8);
        modbus1.Host_Txbuf[length++] = (uint8_t)ModbusData[i];
    }
    taskEXIT_CRITICAL();
    uint16_t crc = Modbus_CRC16(modbus1.Host_Txbuf, length);
    modbus1.Host_Txbuf[length++] = (uint8_t)(crc >> 8);
    modbus1.Host_Txbuf[length++] = (uint8_t)crc;
    modbus1.Host_send_flag = HAL_UART_Transmit(&huart2, modbus1.Host_Txbuf, length, 100U) == HAL_OK;
    (void)osMutexRelease(s_master_tx_mutex);
}
