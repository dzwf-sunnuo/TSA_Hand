#ifndef __APP_COMM_TASKS_H__
#define __APP_COMM_TASKS_H__

#include "main.h"
#include "cmsis_os.h"

/* ================================================================
 *  统一动作指令集 (Action ID)
 *  所有输入源 (IR / CAN / Modbus) 最终都映射为一个 ActionID，
 *  由 Action_Dispatch() 统一路由到对应子系统。
 * ================================================================ */
typedef enum {
    ACTION_NONE = 0,

    /* —— 手腕运动 —— */
    ACTION_WRIST_PENDULUM_START,    // 手腕开始摆动 (变速模式)
    ACTION_WRIST_PENDULUM_STOP,     // 手腕停止摆动
    ACTION_WRIST_ROTATION_START,    // 手腕开始旋转 (定速模式)
    ACTION_WRIST_ROTATION_STOP,     // 手腕停止旋转
    ACTION_WRIST_POS_PRESET1,       // 手腕预设位置 1
    ACTION_WRIST_POS_PRESET2,       // 手腕预设位置 2
    ACTION_WRIST_POS_PRESET3,       // 手腕预设位置 3
    ACTION_WRIST_SET_CYCLE,         // 设置手腕运动周期 (param1=cycle_ms)

    /* —— 手指连续动作 —— */
    ACTION_FINGER_COUNT,            // 数数字 (0-9)
    ACTION_FINGER_BEND,             // 连续弯折
    ACTION_FINGER_GESTURE,          // 其他手势 (OK 等)
    ACTION_FINGER_FIST,             // 连续握拳

    /* —— 手指单个动作 —— */
    ACTION_FINGER_SINGLE,           // 单个手势 (参数指定具体姿势编号)

    /* —— 系统级 —— */
    ACTION_ALL_STOP,                // 全部停止 / 复位

    ACTION_COUNT
} ActionID;

/* 动作参数 (携带单次动作的附加信息) */
typedef struct {
    ActionID action;        // 动作类型
    uint16_t param1;        // 参数1 (如: 循环次数、预设位置编号、单手势编号)
    uint16_t param2;        // 参数2 (如: 速度、延时)
} ActionCmd_t;

/* ================================================================
 *  手腕运动消息队列 (方案 A: 队列驱动的启停控制)
 *  Task_WristMotion 以 osMessageQueueGet(40ms超时) 同时实现
 *  命令接收和 40ms 心跳定时，无需全局标志位。
 * ================================================================ */
typedef enum {
    WRIST_CMD_STOP = 0,     // 停止
    WRIST_CMD_PENDULUM,     // 摆动模式
    WRIST_CMD_ROTATION,     // 旋转模式
} WristCmdType;

typedef struct {
    WristCmdType cmd;       // 命令类型
    uint16_t    cycle_ms;  // 运动周期 (仅 START 时有效)
} WristCmd_t;

/* CAN 消息队列结构体 */
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
} App_CAN_Msg_t;

/* 全局通信任务初始化函数 */
void App_Comm_Init(void);

/* 统一动作分发器 (由 IR/CAN 任务调用) */
void Action_Dispatch(const ActionCmd_t *cmd);

/* IR 按键 → ActionID 映射 (输入: NEC 键值, 输出: 填充 cmd) */
void IR_Key_MapToAction(uint16_t key_code, ActionCmd_t *cmd);

/* CAN 消息 → ActionID 映射 (输入: CAN 消息, 输出: 填充 cmd) */
void CAN_Msg_MapToAction(const App_CAN_Msg_t *msg, ActionCmd_t *cmd);

/* RTOS 任务入口函数声明 */
void Task_ModbusPC_Entry(void *argument);
void Task_SlaveDispatch_Entry(void *argument);
void Task_CAN_Process_Entry(void *argument);
void Task_IR_Process_Entry(void *argument);
void Task_WristMotion_Entry(void *argument);

/* 手腕运动控制 wrapper (由 rs485.c / wrist.c / Action_Dispatch 调用) */
void WristMotion_Start(uint8_t mode);
void WristMotion_Stop(void);

/* 手腕运动命令队列 (供 wrist.c / rs485.c 通过 WristMotion_Start/Stop 写入) */
extern osMessageQueueId_t Queue_WristCmd;

/* 新动作信号量: Action_Dispatch 每次分发时释放, finger_motion 用它检测中断 */
extern osSemaphoreId_t Sem_NewMotionCmd;

#endif /* __APP_COMM_TASKS_H__ */
