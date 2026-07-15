/*
 * Tactile_Sensor.h
 *
 *  创建日期: 2026-06-04
 *  作者: Gemini
 */

#ifndef INC_TACTILE_SENSOR_H_
#define INC_TACTILE_SENSOR_H_

#include "main.h"
#include "spi.h"

/**
 * @brief 触觉传感器句柄结构体
 */
typedef struct {
    SPI_HandleTypeDef *hspi;    // SPI 句柄
    GPIO_TypeDef *cs_port;      // 片选端口
    uint16_t cs_pin;            // 片选引脚
    int16_t offset_x, offset_y, offset_z; // 软件偏置（预留）
    float fx, fy, fz, f_sum;    // 物理力值（单位：N）
    char name[16];              // 传感器名称
    HAL_StatusTypeDef last_status; // 最近一次通信状态
} Tactile_Sensor_t;

/**
 * @brief 微秒级延时工具函数
 * @param us 延时微秒数
 */
void Tactile_DelayUs(uint16_t us);


/**
 * @brief 初始化触觉传感器句柄并触发硬件自动校准
 * @param sensor 传感器句柄指针
 * @param hspi SPI 句柄
 * @param cs_port 片选 GPIO 端口
 * @param cs_pin 片选 GPIO 引脚
 * @param name 传感器名称
 * @return HAL 状态
 */
HAL_StatusTypeDef Tactile_Init(Tactile_Sensor_t *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, const char *name);

/**
 * @brief 更新传感器数据（读取 Fx, Fy, Fz 并转换为牛顿）
 * @param sensor 传感器句柄指针
 * @return HAL 状态
 */
HAL_StatusTypeDef Tactile_Update(Tactile_Sensor_t *sensor);

/**
 * @brief 注册传感器到管理器，以便进行基于任务的自动更新
 * @param sensor 传感器句柄指针
 */
void Tactile_RegisterSensor(Tactile_Sensor_t *sensor);

/**
 * @brief 启动触觉传感器管理的 FreeRTOS 任务
 */
void Tactile_StartManager(void);

#endif /* INC_TACTILE_SENSOR_H_ */
