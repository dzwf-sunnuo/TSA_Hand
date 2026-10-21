#ifndef INC_TACTILE_SENSOR_H_
#define INC_TACTILE_SENSOR_H_

#include "main.h"
#include "spi.h"

// Tactile_Sensor_t：保存单个三轴力传感器的片选、最近结果和采集状态
// 参数：无；字段中的力单位为牛顿，last_update_tick使用HAL毫秒时基
// 返回值：无
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    int16_t offset_x, offset_y, offset_z; // 保留原偏置字段，不改变传感器协议
    float fx, fy, fz, f_sum;
    char name[16];
    HAL_StatusTypeDef last_status;
    uint8_t valid;             // 本次收发成功且响应满足原更新条件时置1
    uint32_t last_update_tick; // 最近真正更新数据的时间，失败不刷新
} Tactile_Sensor_t;

// Tactile_DelayUs：使用1 MHz自由运行计数器等待指定微秒数
// 参数：us - 等待微秒数，支持16位计数器的一次回绕
// 返回值：无
void Tactile_DelayUs(uint16_t us);

// Tactile_Init：绑定SPI与片选并发送原有硬件校准命令
// 参数：sensor - 句柄；hspi - SPI；cs_port/cs_pin - 片选；name - 名称
// 返回值：HAL_OK表示校准命令发送成功，不表示已取得有效力数据
HAL_StatusTypeDef Tactile_Init(Tactile_Sensor_t *sensor, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin, const char *name);

// Tactile_Update：按10/30/30/10微秒时序读取一次三轴力
// 参数：sensor - 待采集句柄，调用者负责独占SPI并串行选择传感器
// 返回值：HAL收发状态；数据是否真正更新还需检查sensor->valid
HAL_StatusTypeDef Tactile_Update(Tactile_Sensor_t *sensor);

// Tactile_RegisterSensor：保留原注册接口，注册不触发采集
// 参数：sensor - 需要登记的传感器句柄
// 返回值：无
void Tactile_RegisterSensor(Tactile_Sensor_t *sensor);

#endif
