#include "Tactile_Sensor.h"
#include "tim.h"
#include <math.h>
#include <string.h>

/* 外部句柄声明 */
extern TIM_HandleTypeDef htim13;

/* 传感器管理器静态变量 */
#define MAX_SENSORS 3
static Tactile_Sensor_t *s_sensors[MAX_SENSORS];
static uint8_t s_sensor_count = 0;


/**
 * @brief 使用 TIM13 的微秒级延时工具
 * @param us 延时微秒数
 */
void Tactile_DelayUs(uint16_t us)
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim13);
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim13) - start) < us);
}



/**
 * @brief 初始化触觉传感器句柄并触发硬件自动校准
 */
HAL_StatusTypeDef Tactile_Init(Tactile_Sensor_t *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, const char *name)
{
    if (sensor == NULL) return HAL_ERROR;

    sensor->hspi = hspi;
    sensor->cs_port = cs_port;
    sensor->cs_pin = cs_pin;
    sensor->offset_x = 0;
    sensor->offset_y = 0;
    sensor->offset_z = 0;
    sensor->fx = 0.0f;
    sensor->fy = 0.0f;
    sensor->fz = 0.0f;
    sensor->f_sum = 0.0f;
    sensor->last_status = HAL_OK;

    if (name != NULL)
    {
        strncpy(sensor->name, name, sizeof(sensor->name) - 1);
        sensor->name[sizeof(sensor->name) - 1] = '\0';
    }

    /* 初始确保 CS 为高电平 */
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    Tactile_DelayUs(80);

    /* 触发传感器硬件自动校准 */
    static const uint8_t cal_frame[] = {0x79, 0x03, 0x00, 0x01, 0x00, 0x01, 0xA7};
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    Tactile_DelayUs(100);
    status = HAL_SPI_Transmit(sensor->hspi, (uint8_t *)cal_frame, sizeof(cal_frame), 1000);
    Tactile_DelayUs(300);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    Tactile_DelayUs(80);

    /* 等待校准完成 */
    Tactile_DelayUs(10000);

    return status;
}

/**
 * @brief 更新传感器数据（读取 Fx, Fy, Fz 并转换为牛顿）
 *
 * 调用频率: 10ms (由 Motor_Control_Loop case 6 驱动)
 * 分辨率: 1 LSB = 0.1 N
 */
HAL_StatusTypeDef Tactile_Update(Tactile_Sensor_t *sensor)
{
    if (sensor == NULL) return HAL_ERROR;

    static const uint8_t read_cmd[] = {0xFB, 0xF0, 0x03, 0x03, 0x00};
    uint8_t rx[5];
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    Tactile_DelayUs(100);

    status = HAL_SPI_Transmit(sensor->hspi, (uint8_t *)read_cmd, sizeof(read_cmd), 1000);
    if (status != HAL_OK) goto end;

    Tactile_DelayUs(300);

    status = HAL_SPI_Receive(sensor->hspi, rx, sizeof(rx), 1000);
    if (status != HAL_OK) goto end;

    Tactile_DelayUs(300);

    if (rx[0] != 0)
    {
        int8_t  raw_fx = (int8_t)rx[1];
        int8_t  raw_fy = (int8_t)rx[2];
        uint8_t raw_fz = rx[3];

        sensor->fx = (float)raw_fx * 0.1f;
        sensor->fy = (float)raw_fy * 0.1f;
        sensor->fz = (float)raw_fz * 0.1f;
        sensor->f_sum = sqrtf(sensor->fx * sensor->fx +
                              sensor->fy * sensor->fy +
                              sensor->fz * sensor->fz);
    }

end:
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    Tactile_DelayUs(80);

    sensor->last_status = status;
    return status;
}

/**
 * @brief 注册传感器到管理器
 */
void Tactile_RegisterSensor(Tactile_Sensor_t *sensor)
{
    if (s_sensor_count < MAX_SENSORS && sensor != NULL)
    {
        s_sensors[s_sensor_count++] = sensor;
    }
}
