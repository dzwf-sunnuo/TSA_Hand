#ifndef __MOVE_H
#define __MOVE_H


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
// 假设指令帧最大长度，可根据实际需求调整
#define MAX_FRAME_LENGTH     40 
// 指令帧结构体
typedef struct {
    uint8_t frame_type; // 帧类型（指令帧或应答帧相关标识）
    uint8_t header1;    // 帧头1
    uint8_t header2;    // 帧头2
    uint8_t data_length; // 数据长度
    uint8_t id_address; // ID地址
    uint8_t cmd_type;   // 指令类型
    uint16_t register_addr; // 寄存器地址
    uint8_t data[MAX_FRAME_LENGTH]; // 数据段
    uint8_t check_sum; // 校验和
} UART_Instruction_Frame;
// 运动参数
#define MAX_POSITION 0x07D0      // 2000 对应 16mm
#define MIN_POSITION 0x0000      // 0 对应 0mm
#define MAX_SPEED 0x0FA0         // 4000 对应 32mm/s
#define MIN_SPEED 0x00F4         // 500 对应 4mm/s (最小工作速度)
#define M_PI      3.14


// 运动参数映射常量
#define POSITION_RANGE 2000.0f    // 位置范围 0-2000
#define SPEED_RANGE 4000.0f       // 速度范围 0-4000
#define PHYSICAL_POS_RANGE 16.0f  // 物理位置范围 16mm
#define PHYSICAL_SPEED_RANGE 32.0f // 物理速度范围 32mm/s

// 固定目标位置定义
// 摆手模式固定位置
#define POS_PENDULUM_LEFT_1    0x0000    // 电机1左位置
#define POS_PENDULUM_LEFT_2    0x07D0    // 电机2左位置
#define POS_PENDULUM_RIGHT_1   0x07D0    // 电机1右位置
#define POS_PENDULUM_RIGHT_2   0x0000    // 电机2右位置

// 旋转模式固定位置
#define POS_ROT_DOWN_1         0x0000    // 全下 - 电机1
#define POS_ROT_DOWN_2         0x0000    // 全下 - 电机2
#define POS_ROT_LEFT_1         0x07D0    // 手腕左 - 电机1
#define POS_ROT_LEFT_2         0x0090    // 手腕左 - 电机2
#define POS_ROT_UP_1           0x07D0    // 全上 - 电机1
#define POS_ROT_UP_2           0x07D0    // 全上 - 电机2
#define POS_ROT_RIGHT_1        0x0000    // 手腕右 - 电机1
#define POS_ROT_RIGHT_2        0x07D0    // 手腕右 - 电机2

//typedef struct {
//    uint16_t target_pos;         // 目标位置（固定）
//    uint16_t current_speed;      // 当前速度
//    uint8_t is_active;           // 电机是否激活
//} MotorController;

typedef struct {
    uint16_t start_pos;      // 起始位置
    uint16_t target_pos;     // 目标位置
    uint16_t current_pos;    // 当前位置
    uint16_t current_speed;  // 当前速度
    uint32_t motion_start_time; // 运动开始时间
    uint8_t is_active;       // 是否激活
} MotorController;

typedef enum {
    MODE_PENDULUM = 1,    // 模式1：摆手运动
    MODE_ROTATION = 2     // 模式2：转动手腕
} WristMode;

typedef enum {
    STATE_DOWN = 0,       // 全下位置
    STATE_LEFT = 1,       // 手腕左位置
    STATE_UP = 2,         // 全上位置
    STATE_RIGHT = 3       // 手腕右位置
} RotationState;


uint8_t calculate_checksum(uint8_t *data, uint8_t length);
void build_instruction_frame(UART_Instruction_Frame *frame, 
                             uint8_t id_addr, uint8_t cmd, uint16_t reg_addr, 
                             uint8_t *data, uint8_t data_len); 
void send_uart_instruction_frame(UART_Instruction_Frame *frame, UART_HandleTypeDef *huart) ;
extern UART_Instruction_Frame fraMotor1;
extern UART_Instruction_Frame fraMotor2;
void vel_move(uint8_t POS_H,uint8_t POS_L,uint8_t VEL_H,uint8_t VEL_L);
void vel_move1(uint8_t POS_H,uint8_t POS_L,uint8_t VEL_H,uint8_t VEL_L);
//void read();


void updateSmartMotionControl(uint8_t mode, uint16_t speed_encoded);

void set_wrist_motion_Can(uint8_t data0,uint8_t data1);

#endif

