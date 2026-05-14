#ifndef __WRIST_H
#define __WRIST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MAX_FRAME_LENGTH     40

/* 串口指令帧结构体 */
typedef struct {
    uint8_t frame_type;     // 帧类型
    uint8_t header1;        // 帧头1
    uint8_t header2;        // 帧头2
    uint8_t data_length;    // 数据长度
    uint8_t id_address;     // ID地址
    uint8_t cmd_type;       // 指令类型
    uint16_t register_addr; // 寄存器地址
    uint8_t data[MAX_FRAME_LENGTH]; // 数据段
    uint8_t check_sum;      // 校验和
} UART_Instruction_Frame;

/* 运动参数范围 */
#define MAX_POSITION 0x07D0      // 2000 对应 16mm
#define MIN_POSITION 0x0000      // 0 对应 0mm
#define MAX_SPEED 0x0FA0         // 4000 对应 32mm/s
#define MIN_SPEED 0x0064         // 100 最小基准速度 (防止死区)

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

/* 摆手模式固定位置 */
#define POS_PENDULUM_LEFT_1    0x0000    // 电机1左位置
#define POS_PENDULUM_LEFT_2    0x07D0    // 电机2左位置
#define POS_PENDULUM_RIGHT_1   0x07D0    // 电机1右位置
#define POS_PENDULUM_RIGHT_2   0x0000    // 电机2右位置

/* 旋转模式固定位置 */
#define POS_ROT_DOWN_1         0x0000    
#define POS_ROT_DOWN_2         0x0000    
#define POS_ROT_LEFT_1         0x07D0    
#define POS_ROT_LEFT_2         0x0090    
#define POS_ROT_UP_1           0x07D0    
#define POS_ROT_UP_2           0x07D0    
#define POS_ROT_RIGHT_1        0x0000    
#define POS_ROT_RIGHT_2        0x07D0    

/* 运动模式 */
typedef enum {
    MODE_PENDULUM = 1,    // 模式1：左右摆手运动
    MODE_ROTATION = 2     // 模式2：上下左右旋转
} WristMode;

/* 旋转状态 */
typedef enum {
    STATE_DOWN = 0,       
    STATE_LEFT = 1,       
    STATE_UP = 2,         
    STATE_RIGHT = 3       
} RotationState;

extern UART_Instruction_Frame fraMotor1;
extern UART_Instruction_Frame fraMotor2;

void vel_move(uint8_t POS_H,uint8_t POS_L,uint8_t VEL_H,uint8_t VEL_L);
void vel_move1(uint8_t POS_H,uint8_t POS_L,uint8_t VEL_H,uint8_t VEL_L);

/* 智能运动控制API (由htim2中断调用) */
void updateSmartMotionControl(uint8_t mode, uint16_t cycle_time_ms);
void set_wrist_motion_Can(uint8_t data0,uint8_t data1);

#ifdef __cplusplus
}
#endif

#endif /* __WRIST_H */
