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

// ModbusRequest_t：保存本次请求上下文，两路接收缓存独立，只有寄存器和业务逻辑共用。
// 参数：data为调用期间稳定的请求；length含CRC；reply仅控制USART1应答；response为本次局部回复缓存。
// 返回值：请求上下文类型，只在统一的ModbusPC任务栈上创建。
typedef struct {
    const uint8_t *data;
    uint16_t length;
    bool reply;
    uint8_t response[100];
} ModbusRequest_t;

static bool Modbus_Func3(ModbusRequest_t *request);
static bool Modbus_Func6(ModbusRequest_t *request);
static bool Modbus_Func16(ModbusRequest_t *request);

// Modbus_ReadU16：读取协议中的两字节大端整数
// 参数：data - 两字节起始地址
// 返回值：无符号16位数值
static uint16_t Modbus_ReadU16(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

// Modbus_SendResponse：追加沿用项目CRC编码的校验值并整帧回复
// 参数：request - 本次请求上下文；length - 不含CRC的响应长度
// 返回值：无，UART4静默请求和广播写入均不产生响应
static void Modbus_SendResponse(ModbusRequest_t *request, uint16_t length)
{
    if (!request->reply || request->data[0] == 0U ||
        length + 2U > sizeof(request->response)) return;
    uint16_t crc = Modbus_CRC16(request->response, length);
    request->response[length++] = (uint8_t)(crc >> 8);
    request->response[length++] = (uint8_t)crc;
    (void)HAL_UART_Transmit(&huart1, request->response, length, 20U);
}

// Modbus_SendException：回复功能码异常、地址异常或数据异常
// 参数：request - 本次请求上下文；code - MODBUS异常码
// 返回值：false表示请求被拒绝；UART4和广播帧不发送异常回复
static bool Modbus_SendException(ModbusRequest_t *request, uint8_t code)
{
    request->response[0] = modbus2.myadd;
    request->response[1] = request->data[1] | 0x80U;
    request->response[2] = code;
    Modbus_SendResponse(request, 3U);
    return false;
}

// Modbus_RangeOverlaps：判断写入区间是否触及某一业务寄存器区间
// 参数：start、count - 写入范围；first、last - 闭区间业务范围
// 返回值：true表示有交集
static bool Modbus_RangeOverlaps(uint16_t start, uint16_t count,
                                 uint16_t first, uint16_t last)
{
    return start <= last && (uint32_t)start + count > first;
}

// Modbus_ValidateValue：校验上报配置及新增统一导纳参数的可写值
// 参数：address - 寄存器地址；value - 候选值
// 返回值：true表示允许写入；上报阈值保留原范围，新增导纳参数要求为正整数
static bool Modbus_ValidateValue(uint16_t address, uint16_t value)
{
    if (address == UART4_REPORT_REG_ENABLE) return value <= 1U;
    if (address == UART4_REPORT_REG_FREQUENCY) {
        return value >= 1U && value <= UART4_REPORT_MAX_FREQUENCY;
    }
    // 仅约束新增参数区，避免零刚度或零阻尼配置，同时不改变任何原控制命令的校验规则。
    if (address >= ADM_REG_MASTER_K_CONTACT && address <= ADM_REG_MASTER_DAMP) {
        return value != 0U;
    }
    return true;
}

// Modbus_ApplyWrite：按已提交的地址范围触发控制或上报配置通知
// 参数：start - 写入首地址；count - 已成功写入的寄存器数
// 返回值：无，配置写入及读取不会重复下发任何电机动作
static void Modbus_ApplyWrite(uint16_t start, uint16_t count)
{
    uint8_t boards = 0U;
    if (Modbus_RangeOverlaps(start, count, HAND_CONTROL_BASE, HAND_BOARD2_BASE - 1U)) boards |= 0x01U;
    if (Modbus_RangeOverlaps(start, count, HAND_BOARD2_BASE, HAND_CONTROL_BASE + HAND_CONTROL_COUNT - 1U)) boards |= 0x02U;
    if (boards != 0U) App_RequestSlaveDispatch(boards);

    // 参数事件独立于运动事件，单独调参不会重发目标或模式，也不会解除从板超力锁定。
    if (Modbus_RangeOverlaps(start, count, ADM_REG_MASTER_K_CONTACT, ADM_REG_MASTER_DAMP)) {
        App_RequestAdmittanceDispatch();
    }

    bool control_changed = Modbus_RangeOverlaps(start, count, WRIST_REG_CONTROL, WRIST_REG_CONTROL);
    if ((Reg[WRIST_REG_CONTROL] & 0xFFU) == 1U) {
        if (control_changed || Modbus_RangeOverlaps(start, count, 0U, 1U)) {
            vel_move((uint8_t)(Reg[0] >> 8), (uint8_t)Reg[0],
                     (uint8_t)(Reg[1] >> 8), (uint8_t)Reg[1]);
        }
        if (control_changed || Modbus_RangeOverlaps(start, count, 2U, 3U)) {
            vel_move1((uint8_t)(Reg[2] >> 8), (uint8_t)Reg[2],
                      (uint8_t)(Reg[3] >> 8), (uint8_t)Reg[3]);
        }
    }
    if (control_changed || Modbus_RangeOverlaps(start, count, WRIST_REG_CYCLE, WRIST_REG_CYCLE)) {
        cycle_time = Reg[WRIST_REG_CYCLE];
        if ((Reg[WRIST_REG_CONTROL] >> 8) != 0U) WristMotion_Start(MODE_PENDULUM);
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
    Reg[WRIST_REG_CONTROL] = 0x0001U;
    Reg[WRIST_REG_CYCLE] = 0x0898U; // 保留原来的2200毫秒手腕摆动周期。
    Reg[WRIST_REG_PRESET1] = 0x012CU;
    Reg[WRIST_REG_PRESET2] = 0x06A4U;
    Reg[UART4_REPORT_REG_ENABLE] = UART4_REPORT_DEFAULT_ENABLE;
    Reg[UART4_REPORT_REG_FREQUENCY] = UART4_REPORT_DEFAULT_FREQUENCY;
    Reg[UART4_REPORT_REG_RATIO] = UART4_REPORT_DEFAULT_RATIO;
    Reg[UART4_REPORT_REG_FORCE_MIN] = UART4_REPORT_DEFAULT_FORCE_MIN;
    // 只初始化主板缓存，不在启动时额外发送命令；首次参数写入才向两从板同步完整三项。
    Reg[ADM_REG_MASTER_K_CONTACT] = ADM_DEFAULT_K_CONTACT;
    Reg[ADM_REG_MASTER_K_RETURN] = ADM_DEFAULT_K_RETURN;
    Reg[ADM_REG_MASTER_DAMP] = ADM_DEFAULT_DAMP;
    modbus2.myadd = 0x10U;
}

// Modbus_IsRequestFrame：校验地址、长度和项目原有CRC编码，供两路入口共同使用
// 参数：data - 含CRC的稳定请求；length - 接收字节数
// 返回值：true表示基础校验通过；CRC函数内部只读取数据
bool Modbus_IsRequestFrame(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length < 4U || length > sizeof(modbus2.rcbuf)) return false;
    if (data[0] != modbus2.myadd && data[0] != 0U) return false;
    return Modbus_CRC16((uint8_t *)data, length - 2U) ==
           Modbus_ReadU16(&data[length - 2U]);
}

// Modbus_ProcessRequest：按来源选择是否回复，共用寄存器校验及业务执行
// 参数：data - 请求数据；length - 含CRC长度；reply - true回复USART1，false完全静默
// 返回值：true表示合法并完成处理，false表示CRC、功能码或参数失败；只由ModbusPC任务调用
bool Modbus_ProcessRequest(const uint8_t *data, uint16_t length, bool reply)
{
    if (!Modbus_IsRequestFrame(data, length)) return false;
    // 本次上下文在控制任务栈上，不覆盖USART1仍由DMA持有的接收缓存。
    ModbusRequest_t request = {.data = data, .length = length, .reply = reply};
    switch (data[1]) {
    case 0x03U: return data[0] == 0U ? true : Modbus_Func3(&request);
    case 0x06U: return Modbus_Func6(&request);
    case 0x10U: return Modbus_Func16(&request);
    default: return Modbus_SendException(&request, 0x01U);
    }
}

// modbus2_Event：把USART1交付的请求送入公共入口，保留正常应答
// 参数：无，DMA在本次处理后重新启动，不会覆盖正在解析的数据
// 返回值：无；错误请求和读取不会触发控制动作
void modbus2_Event(void)
{
    if (modbus2.reflag == 0U) return;
    (void)Modbus_ProcessRequest(modbus2.rcbuf, modbus2.recount, true);
    modbus2.recount = 0U;
    modbus2.reflag = 0U;
}

// Modbus_Func3：读取共用寄存器，UART4只验证并消费请求，不发送读取结果
// 参数：request - 本次请求上下文，长度必须为8字节
// 返回值：true表示合法，false表示数量或地址错误
static bool Modbus_Func3(ModbusRequest_t *request)
{
    if (request->length != 8U) return Modbus_SendException(request, 0x03U);
    uint16_t start = Modbus_ReadU16(&request->data[2]);
    uint16_t count = Modbus_ReadU16(&request->data[4]);
    if (count == 0U || 5U + 2U * (uint32_t)count > sizeof(request->response))
        return Modbus_SendException(request, 0x03U);
    if ((uint32_t)start + count > MODBUS_REG_COUNT)
        return Modbus_SendException(request, 0x02U);
    if (!request->reply) return true;
    request->response[0] = modbus2.myadd;
    request->response[1] = 0x03U;
    request->response[2] = (uint8_t)(count * 2U);
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < count; ++i) {
        request->response[3U + 2U * i] = (uint8_t)(Reg[start + i] >> 8);
        request->response[4U + 2U * i] = (uint8_t)Reg[start + i];
    }
    taskEXIT_CRITICAL();
    Modbus_SendResponse(request, 3U + 2U * count);
    return true;
}

// Modbus_Func6：校验单个写入值，按来源选择应答并触发原控制逻辑
// 参数：request - 当前请求上下文，长度必须为8字节
// 返回值：true表示写入成功，false表示参数错误或配置锁不可用
static bool Modbus_Func6(ModbusRequest_t *request)
{
    if (request->length != 8U) return Modbus_SendException(request, 0x03U);
    uint16_t start = Modbus_ReadU16(&request->data[2]);
    uint16_t value = Modbus_ReadU16(&request->data[4]);
    if (start >= MODBUS_REG_COUNT) return Modbus_SendException(request, 0x02U);
    if (!Modbus_ValidateValue(start, value)) return Modbus_SendException(request, 0x03U);
    bool telemetry_config = Modbus_RangeOverlaps(start, 1U,
        UART4_REPORT_REG_ENABLE, UART4_REPORT_REG_FORCE_MIN);
    // 必须先取任务锁再进入短临界区，不在全局关中断期间等待。
    if (telemetry_config && !Uart4Telemetry_LockConfig())
        return Modbus_SendException(request, 0x04U);
    taskENTER_CRITICAL();
    Reg[start] = value;
    taskEXIT_CRITICAL();
    if (telemetry_config) Uart4Telemetry_UnlockConfig();
    memcpy(request->response, request->data, 6U);
    Modbus_SendResponse(request, 6U);
    Modbus_ApplyWrite(start, 1U);
    return true;
}

// Modbus_Func16：预校验整帧后原子提交全部寄存器，禁止部分成功
// 参数：request - 本次请求，请求长度必须等于9加两倍寄存器数量
// 返回值：true表示全部写入成功，false表示请求被拒绝
static bool Modbus_Func16(ModbusRequest_t *request)
{
    if (request->length < 9U) return Modbus_SendException(request, 0x03U);
    uint16_t start = Modbus_ReadU16(&request->data[2]);
    uint16_t count = Modbus_ReadU16(&request->data[4]);
    uint32_t bytes = 2U * (uint32_t)count;
    if (count == 0U || bytes + 9U != request->length || bytes != request->data[6])
        return Modbus_SendException(request, 0x03U);
    if ((uint32_t)start + count > MODBUS_REG_COUNT)
        return Modbus_SendException(request, 0x02U);
    for (uint16_t i = 0U; i < count; ++i) {
        if (!Modbus_ValidateValue(start + i, Modbus_ReadU16(&request->data[7U + 2U * i])))
            return Modbus_SendException(request, 0x03U);
    }
    bool telemetry_config = Modbus_RangeOverlaps(start, count,
        UART4_REPORT_REG_ENABLE, UART4_REPORT_REG_FORCE_MIN);
    if (telemetry_config && !Uart4Telemetry_LockConfig())
        return Modbus_SendException(request, 0x04U);
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < count; ++i)
        Reg[start + i] = Modbus_ReadU16(&request->data[7U + 2U * i]);
    taskEXIT_CRITICAL();
    if (telemetry_config) Uart4Telemetry_UnlockConfig();
    memcpy(request->response, request->data, 6U);
    Modbus_SendResponse(request, 6U);
    Modbus_ApplyWrite(start, count);
    return true;
}

// modbus2_Func3：兼容原USART1读取接口，转交公共处理器
// 参数：无，使用USART1已校验接收缓存
// 返回值：无
void modbus2_Func3(void)
{
    ModbusRequest_t request = {.data = modbus2.rcbuf, .length = modbus2.recount, .reply = true};
    (void)Modbus_Func3(&request);
}

// modbus2_Func6：兼容原USART1单寄存器写入接口
// 参数：无，使用USART1已校验接收缓存
// 返回值：无
void modbus2_Func6(void)
{
    ModbusRequest_t request = {.data = modbus2.rcbuf, .length = modbus2.recount, .reply = true};
    (void)Modbus_Func6(&request);
}

// modbus2_Func16：兼容原USART1多寄存器写入接口
// 参数：无，使用USART1已校验接收缓存
// 返回值：无
void modbus2_Func16(void)
{
    ModbusRequest_t request = {.data = modbus2.rcbuf, .length = modbus2.recount, .reply = true};
    (void)Modbus_Func16(&request);
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

// Host_write_legacy_action：只为固件内部旧动作表提供格式转换，目标与模式成对下发
// 参数：slave - 地址1或2；legacy - 四路旧目标和一个公共模式
// 返回值：无，原motor0的二号板占位不会出现在新帧中
void Host_write_legacy_action(uint8_t slave, const uint16_t legacy[5])
{
    if (legacy == NULL || (slave != 1U && slave != 2U)) return;
    uint16_t commands[HAND_BOARD1_COUNT] = {0};
    uint8_t first = slave == 1U ? 0U : 1U;
    uint8_t count = slave == 1U ? 4U : 3U;
    for (uint8_t slot = 0U; slot < count; ++slot) {
        commands[2U * slot] = legacy[first + slot];
        commands[2U * slot + 1U] = legacy[4];
    }
    Host_write16_slave(slave, 0x10U, 0U, (uint16_t)(2U * count),
        (uint8_t)(4U * count), commands);
}
