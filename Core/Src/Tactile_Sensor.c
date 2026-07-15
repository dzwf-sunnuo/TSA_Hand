/*
 * Tactile_Sensor.c
 *
 *  创建日期: 2026-06-04
 *  作者: Gemini
 */

#include "Tactile_Sensor.h"
#include "tim.h"
#include "cmsis_os2.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* 外部句柄声明 */
extern TIM_HandleTypeDef htim13;
extern UART_HandleTypeDef huart1;

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
 * @param sensor 传感器句柄指针
 * @param hspi SPI 句柄
 * @param cs_port 片选 GPIO 端口
 * @param cs_pin 片选 GPIO 引脚
 * @param name 传感器名称
 * @return HAL 状态
 */
HAL_StatusTypeDef Tactile_Init(Tactile_Sensor_t *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, const char *name)
{
    if (sensor == NULL) return HAL_ERROR;

    /* 绑定硬件参数 */
    sensor->hspi = hspi;
    sensor->cs_port = cs_port;
    sensor->cs_pin = cs_pin;

    /* 初始化数据字段 */
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

    /* 触发传感器硬件自动校准：发送硬编码写命令帧
     * 79 = 功能码 0x79, bit7=0 写操作
     * 03 00 = 地址 3 (小端)
     * 01 00 = 长度 1 (小端)
     * 01 = 数据：写 1 触发校准
     * A7 = CRC8(79 03 00 01 00 01)
     */
    static const uint8_t cal_frame[] = {0x79, 0x03, 0x00, 0x01, 0x00, 0x01, 0xA7};
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    Tactile_DelayUs(100);
    status = HAL_SPI_Transmit(sensor->hspi, (uint8_t *)cal_frame, sizeof(cal_frame), 1000);
    Tactile_DelayUs(300);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    Tactile_DelayUs(80);

    /* 延时 10ms 以等待校准完成 */
    Tactile_DelayUs(10000);

    return status;
}

/**
 * @brief 更新传感器数据（读取 Fx, Fy, Fz 并转换为牛顿）
 * @param sensor 传感器句柄指针
 * @return HAL 状态
 *
 * 读请求 5 字节：FB F0 03 03 00
 *   FB = 功能码 0x7B | bit7=1 读操作
 *   F0 03 = 起始地址 1008 (小端，1008 = 0x03F0)
 *   03 00 = 读取长度 3 (小端)
 * 响应 5 字节：状态 + Fx + Fy + Fz + CRC8
 *   状态字节非零 → 数据有效
 */
HAL_StatusTypeDef Tactile_Update(Tactile_Sensor_t *sensor)
{
    if (sensor == NULL) return HAL_ERROR;

    /* 硬编码读请求：读地址 1008，长度 3 */
    static const uint8_t read_cmd[] = {0xFB, 0xF0, 0x03, 0x03, 0x00};
    uint8_t rx[5]; /* 状态(1) + Fx(1) + Fy(1) + Fz(1) + CRC8(1) */
    HAL_StatusTypeDef status;

    /* CS 拉低，发送读请求 */
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    Tactile_DelayUs(100);

    status = HAL_SPI_Transmit(sensor->hspi, (uint8_t *)read_cmd, sizeof(read_cmd), 1000);
    if (status != HAL_OK) goto end;

    /* 等待传感器准备数据 */
    Tactile_DelayUs(300);

    /* 接收响应 */
    status = HAL_SPI_Receive(sensor->hspi, rx, sizeof(rx), 1000);
    if (status != HAL_OK) goto end;

    Tactile_DelayUs(300);

    /* 状态字节非零 → 数据有效 */
    if (rx[0] != 0)
    {
        int8_t  raw_fx = (int8_t)rx[1];
        int8_t  raw_fy = (int8_t)rx[2];
        uint8_t raw_fz = rx[3];

        /* 分辨率: 1 LSB = 0.1 N */
        sensor->fx = (float)raw_fx * 0.1f;
        sensor->fy = (float)raw_fy * 0.1f;
        sensor->fz = (float)raw_fz * 0.1f;

        /* 计算合力 */
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
 * @brief 以 100Hz 频率更新传感器数据的任务
 */
static void Tactile_Update_Task(void *argument)
{
    for (;;)
    {
        for (uint8_t i = 0; i < s_sensor_count; i++)
        {
            Tactile_Update(s_sensors[i]);
        }
        osDelay(50); // 20Hz
    }
}

/**
 * @brief 每 500ms 通过串口报告传感器数据的任务
 */
static void Tactile_Report_Task(void *argument)
{
    char msg[128];
    for (;;)
    {
        for (uint8_t i = 0; i < s_sensor_count; i++)
        {
            Tactile_Sensor_t *s = s_sensors[i];

            if (s->last_status == HAL_OK)
            {
                // 使用整数模拟浮点数打印，防止环境不支持 %f
                int32_t fx10 = (int32_t)(s->fx * 10.0f);
                int32_t fy10 = (int32_t)(s->fy * 10.0f);
                int32_t fz10 = (int32_t)(s->fz * 10.0f);
                int32_t fs10 = (int32_t)(s->f_sum * 10.0f);

                snprintf(msg, sizeof(msg),
                    "[%s] Fx:%ld.%dN, Fy:%ld.%dN, Fz:%ld.%dN, Fall%ld.%dN\r\n",
                    s->name,
                    fx10/10, (int)labs(fx10%10),
                    fy10/10, (int)labs(fy10%10),
                    fz10/10, (int)labs(fz10%10),
                    fs10/10, (int)labs(fs10%10));
            }
            else
            {
                snprintf(msg, sizeof(msg),
                    "[%s] error:%d\r\n",
                    s->name, s->last_status);
            }

            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }
        osDelay(100);
    }
}

/**
 * @brief 启动触觉传感器管理的 FreeRTOS 任务
 */
void Tactile_StartManager(void)
{
    const osThreadAttr_t updateTask_attributes = {
        .name = "TactileUpdate",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    const osThreadAttr_t reportTask_attributes = {
        .name = "TactileReport",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityLow,
    };

    osThreadNew(Tactile_Update_Task, NULL, &updateTask_attributes);
    osThreadNew(Tactile_Report_Task, NULL, &reportTask_attributes);
}

/**
 * @brief 注册传感器到管理器，以便进行基于任务的自动更新
 */
void Tactile_RegisterSensor(Tactile_Sensor_t *sensor)
{
    if (s_sensor_count < MAX_SENSORS && sensor != NULL)
    {
        s_sensors[s_sensor_count++] = sensor;
    }
}
