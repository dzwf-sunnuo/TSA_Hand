#include "Tactile_Sensor.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

#define TACTILE_IO_TIMEOUT_MS 2U
#define TACTILE_CS_WAIT_US 10U
#define TACTILE_DATA_WAIT_US 30U
#define MAX_SENSORS 3U

static Tactile_Sensor_t *s_sensors[MAX_SENSORS];
static uint8_t s_sensor_count;

// Tactile_DelayUs：按真实微秒等待，不清零共享自由运行计数器
// 参数：us - 等待微秒数，0表示立即返回
// 返回值：无
void Tactile_DelayUs(uint16_t us)
{
    if (us == UINT16_MAX) {
        // 最大参数拆成两段，避免16位差值不可能大于65535而永久等待。
        Tactile_DelayUs(32768U);
        Tactile_DelayUs(32767U);
        return;
    }
    uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim13);
    // 多等待一个计数，抵消读取时的亚微秒相位，保证不早于要求结束。
    if (us != 0U) {
        while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim13) - start) <= us) {
        }
    }
}

// Tactile_RemainingTimeout：共用一次事务的毫秒预算，避免收发累计完整超时
// 参数：start_tick - 本次事务开始的HAL时刻
// 返回值：剩余毫秒数，0表示预算已用尽
static uint32_t Tactile_RemainingTimeout(uint32_t start_tick)
{
    uint32_t elapsed = HAL_GetTick() - start_tick;
    return elapsed < TACTILE_IO_TIMEOUT_MS ? TACTILE_IO_TIMEOUT_MS - elapsed : 0U;
}

// Tactile_WaitIdle：在有限预算内等待最后一位实际移出SPI
// 参数：hspi - SPI句柄；start_tick - 本次事务开始时刻
// 返回值：HAL_OK表示SPI空闲，HAL_TIMEOUT表示预算耗尽
static HAL_StatusTypeDef Tactile_WaitIdle(SPI_HandleTypeDef *hspi, uint32_t start_tick)
{
    while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) != RESET) {
        if (Tactile_RemainingTimeout(start_tick) == 0U) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

// Tactile_RecoverSpi：仅在本模块独占SPI且阻塞事务失败后恢复外设
// 参数：hspi - 失败的SPI句柄，本项目只使用SPI3
// 返回值：无；不调用带长时间等待的HAL_SPI_Abort
static void Tactile_RecoverSpi(SPI_HandleTypeDef *hspi)
{
    __HAL_SPI_DISABLE(hspi);
    if (hspi->Instance == SPI3) {
        // 外设复位确保故障后SCK停止，再恢复原先的模式和速率。
        __HAL_RCC_SPI3_FORCE_RESET();
        __HAL_RCC_SPI3_RELEASE_RESET();
    }
    hspi->State = HAL_SPI_STATE_READY;
    hspi->Lock = HAL_UNLOCKED;
    hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    (void)HAL_SPI_Init(hspi);
}

// Tactile_FinishTransaction：统一完成正常与异常路径的片选收尾
// 参数：sensor - 当前传感器；status - 本次事务状态
// 返回值：无
static void Tactile_FinishTransaction(Tactile_Sensor_t *sensor, HAL_StatusTypeDef status)
{
    if (status != HAL_OK) {
        Tactile_RecoverSpi(sensor->hspi);
    }
    Tactile_DelayUs(TACTILE_DATA_WAIT_US); // 读rd3或写wd2，从最后一位结束后计算
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    Tactile_DelayUs(TACTILE_CS_WAIT_US);   // 读rd4或写wd3，下一事务前保持片选为高
    sensor->last_status = status;
}

// Tactile_Init：初始化句柄，按官方写时序发送原校准命令
// 参数：sensor - 句柄；hspi - SPI；cs_port/cs_pin - 片选；name - 名称
// 返回值：校准命令的HAL发送状态；有效力标志保持为0
HAL_StatusTypeDef Tactile_Init(Tactile_Sensor_t *sensor, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin, const char *name)
{
    static const uint8_t cal_frame[] = {0x79, 0x03, 0x00, 0x01, 0x00, 0x01, 0xA7};
    if (sensor == NULL || hspi == NULL || cs_port == NULL) {
        return HAL_ERROR;
    }
    memset(sensor, 0, sizeof(*sensor));
    sensor->hspi = hspi;
    sensor->cs_port = cs_port;
    sensor->cs_pin = cs_pin;
    if (name != NULL) {
        strncpy(sensor->name, name, sizeof(sensor->name) - 1U);
    }
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    Tactile_DelayUs(TACTILE_CS_WAIT_US);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    Tactile_DelayUs(TACTILE_CS_WAIT_US);
    uint32_t start_tick = HAL_GetTick();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, (uint8_t *)cal_frame,
                                               sizeof(cal_frame), TACTILE_IO_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = Tactile_WaitIdle(hspi, start_tick);
    }
    Tactile_FinishTransaction(sensor, status);
    Tactile_DelayUs(10000U); // 独立保留校准完成等待，不替换成协议中的10/30微秒
    return status;
}

// Tactile_Update：在同一次CS低电平内完成命令发送和响应读取
// 参数：sensor - 已绑定并由采集任务独占的传感器句柄
// 返回值：HAL状态；响应未满足原更新条件时HAL_OK但valid为0
HAL_StatusTypeDef Tactile_Update(Tactile_Sensor_t *sensor)
{
    static const uint8_t read_cmd[] = {0xFB, 0xF0, 0x03, 0x03, 0x00};
    uint8_t rx[5] = {0};
    if (sensor == NULL || sensor->hspi == NULL || sensor->cs_port == NULL) {
        return HAL_ERROR;
    }
    sensor->valid = 0U;
    uint32_t start_tick = HAL_GetTick();
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    Tactile_DelayUs(TACTILE_CS_WAIT_US); // rd1
    HAL_StatusTypeDef status = HAL_SPI_Transmit(sensor->hspi, (uint8_t *)read_cmd,
                                               sizeof(read_cmd), TACTILE_IO_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = Tactile_WaitIdle(sensor->hspi, start_tick);
    }
    if (status == HAL_OK) {
        Tactile_DelayUs(TACTILE_DATA_WAIT_US); // rd2，片选保持为低
        uint32_t remaining = Tactile_RemainingTimeout(start_tick);
        status = remaining == 0U ? HAL_TIMEOUT :
                 HAL_SPI_Receive(sensor->hspi, rx, sizeof(rx), remaining);
        if (status == HAL_OK) {
            status = Tactile_WaitIdle(sensor->hspi, start_tick);
        }
    }
    Tactile_FinishTransaction(sensor, status);
    if (status == HAL_OK && rx[0] != 0U) {
        // 保留原有响应解释，不据时序截图猜测状态位、帧尾或额外校验字段。
        sensor->fx = (float)(int8_t)rx[1] * 0.1f;
        sensor->fy = (float)(int8_t)rx[2] * 0.1f;
        sensor->fz = (float)rx[3] * 0.1f;
        sensor->f_sum = sqrtf(sensor->fx * sensor->fx + sensor->fy * sensor->fy +
                              sensor->fz * sensor->fz);
        sensor->last_update_tick = HAL_GetTick();
        sensor->valid = 1U;
    }
    return status;
}

// Tactile_RegisterSensor：兼容原注册接口，防止重复登记和数组越界
// 参数：sensor - 需要登记的句柄
// 返回值：无
void Tactile_RegisterSensor(Tactile_Sensor_t *sensor)
{
    if (sensor == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < s_sensor_count; i++) {
        if (s_sensors[i] == sensor) {
            return;
        }
    }
    if (s_sensor_count < MAX_SENSORS) {
        s_sensors[s_sensor_count++] = sensor;
    }
}
