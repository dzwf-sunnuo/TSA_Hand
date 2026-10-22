#include "app_comm_tasks.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "can.h"
#include "rs485.h"
#include "Infrared1838.h"
#include "wrist.h"
#include "motion.h"
#include "MyCan.h"
#include "slave_feedback.h"
#include "uart4_telemetry.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* CAN 通信常量 */
#define CAN_HAND_ID             0x020   // 手部控制 CAN ID: 0x020=右手, 0x021=左手 (二选一)
#define CAN_HEARTBEAT_REQUEST   0x00    // data[0]=0x00: 心跳请求
#define CAN_HEARTBEAT_RESPONSE  0x01    // data[0]=0x01: 心跳应答

/* wrist.c 中的外部变量 (配置参数 / 算法状态) */
extern volatile uint16_t cycle_time;
extern volatile uint16_t cycle_time_R;
extern volatile uint8_t  motion_active;

/* ================== RTOS 对象句柄 ================== */
osThreadId_t TaskModbusPCHandle;
osThreadId_t TaskSlaveDispatchHandle;
osThreadId_t TaskCANHandle;
osThreadId_t TaskIRHandle;

osSemaphoreId_t Sem_Uart1_RxIdle;
osSemaphoreId_t Sem_NewSlaveCmd;
osSemaphoreId_t Sem_IR_Ready;
osSemaphoreId_t Sem_NewMotionCmd;      // finger_motion 打断信号
osMessageQueueId_t Queue_CAN_Rx;
osMessageQueueId_t Queue_WristCmd;   // 手腕命令
static uint8_t s_slave_dispatch_mask;
static volatile uint8_t s_pc_rx_error;

/* ================== 系统全局状态 ================== */
volatile uint8_t  sys_infrared_enable = 1;       // 红外遥控使能

/* ================== 统一动作分发器 ================== */

/**
 * @brief 将 ActionCmd 路由到对应的硬件/业务逻辑
 * @note  这是所有输入源 (IR / CAN / Modbus) 的最终汇合点。
 *        当前为框架代码，具体实现留空 (TODO)。
 */
void Action_Dispatch(const ActionCmd_t *cmd)
{
    if (cmd == NULL || cmd->action == ACTION_NONE) return;

    switch (cmd->action) {

    /* —— 手腕运动 —— */
    case ACTION_WRIST_PENDULUM_START:
        WristMotion_Start(MODE_PENDULUM);
        break;

    case ACTION_WRIST_PENDULUM_STOP:
        WristMotion_Stop();
        break;

    case ACTION_WRIST_ROTATION_START:
        WristMotion_Start(MODE_ROTATION);
        break;

    case ACTION_WRIST_ROTATION_STOP:
        WristMotion_Stop();
        break;

    case ACTION_WRIST_POS_PRESET1:
    case ACTION_WRIST_POS_PRESET2:
    case ACTION_WRIST_POS_PRESET3:
        set_wrist_motion_Can(0x10, (uint8_t)cmd->param1);
        break;

    case ACTION_WRIST_SET_CYCLE:
        cycle_time = cmd->param1;   // CAN 0x10-0x05 / 0x11-0x05 下发
        // 同时更新旋转周期，保持两端一致 (也可按需分开)
        cycle_time_R = cmd->param1;
        break;

    /* —— 手指连续动作 —— */
    case ACTION_FINGER_COUNT:
        for (uint16_t r = 0; r < cmd->param1; r++) {
            finger_motion(COUNT_NUMBERS, cmd->param2);
            if (osSemaphoreAcquire(Sem_NewMotionCmd, 0) == osOK) break;
        }
        break;

    case ACTION_FINGER_BEND:
        for (uint16_t r = 0; r < cmd->param1; r++) {
            finger_motion(CONTINUOUS_BENDING, cmd->param2);
            if (osSemaphoreAcquire(Sem_NewMotionCmd, 0) == osOK) break;
        }
        break;

    case ACTION_FINGER_GESTURE:
        for (uint16_t r = 0; r < cmd->param1; r++) {
            finger_motion(OTHER_MOTION, cmd->param2);
            if (osSemaphoreAcquire(Sem_NewMotionCmd, 0) == osOK) break;
        }
        break;

    case ACTION_FINGER_FIST:
        for (uint16_t r = 0; r < cmd->param1; r++) {
            finger_motion(CONTINUOUS_FISTING, cmd->param2);
            if (osSemaphoreAcquire(Sem_NewMotionCmd, 0) == osOK) break;
        }
        break;

    /* —— 手指单个动作 —— */
    case ACTION_FINGER_SINGLE:
        finger_single_action((uint16_t)cmd->param1);
        break;

    /* —— 系统级 —— */
    case ACTION_ALL_STOP:
        // TODO: 全部停止
        WristMotion_Stop();
        // 手指停止:
        // uint16_t stop[5] = {0x0064, 0x0064, 0x1064, 0x0064, 0x0101};
        // Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, stop);
        // osDelay(100);
        // Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, stop);
        break;

    default:
        break;
    }

    // 任何新指令都通知 finger_motion 打断当前动画
    osSemaphoreRelease(Sem_NewMotionCmd);
}

/* ================================================================
 *  输入映射函数：IR 按键 → ActionCmd
 * ================================================================ */

/**
 * @brief 将红外遥控器 NEC 键值映射为统一动作指令
 * @param key_code  NEC 协议解析出的按键值 (来自 InfraredRemote_Get_KeyNum)
 * @param cmd       [输出] 填充的动作指令
 *
 * 野火白色遥控器键值定义 (Infrared1838.h):
 *   IR_1=0x30  IR_2=0x18  IR_3=0x7A  IR_4=0x10
 *   IR_5=0x38  IR_6=0x5A  IR_7=0x42  IR_8=0x4A
 *   IR_UP=0x02 IR_Down=0x98 IR_Left=0xE0 IR_Right=0x90
 *   IR_Xing=0xE2  IR_Empty=0xFF
 */
void IR_Key_MapToAction(uint16_t key_code, ActionCmd_t *cmd)
{
    cmd->action = ACTION_NONE;
    cmd->param1 = 0;
    cmd->param2 = 0;

    switch (key_code) {

    /* —— 手腕控制 —— */
    case IR_1:   // 按键1: 手腕摆动 (变速)
        cmd->action = ACTION_WRIST_PENDULUM_START;
        break;

    case IR_2:   // 按键2: 手腕旋转 (定速)
        cmd->action = ACTION_WRIST_ROTATION_START;
        break;

    /* —— 手指连续动作 —— */
    case IR_3:   // 按键3: 数数字
        cmd->action = ACTION_FINGER_COUNT;
        cmd->param1 = 3;    // 默认循环 3 次
        cmd->param2 = 1300; // 速度参数
        break;

    case IR_4:   // 按键4: 连续弯折手指
        cmd->action = ACTION_FINGER_BEND;
        cmd->param1 = 3;    // 默认循环 3 次
        cmd->param2 = 300;  // 速度参数
        break;

    case IR_5:   // 按键5: 其他手势
        cmd->action = ACTION_FINGER_GESTURE;
        cmd->param1 = 3;
        cmd->param2 = 1000;
        break;

    case IR_7:   // 按键7: 连续握拳
        cmd->action = ACTION_FINGER_FIST;
        cmd->param1 = 3;
        cmd->param2 = 1500;
        break;

    /* —— 系统复位 —— */
    case IR_8:   // 按键8: 全部停止/复位
        cmd->action = ACTION_ALL_STOP;
        break;

    /* —— 单个手势 (IR_Xing 配合方向键选姿势) —— */
    case IR_Xing:   // 星号键: 触发单次手势
        cmd->action = ACTION_FINGER_SINGLE;
        // cmd->param1 由外部设定 (例如 can_num)
        break;

    /* —— 预留: 方向键可用于选择/微调 —— */
    case IR_UP:
    case IR_Down:
    case IR_Left:
    case IR_Right:
        // TODO: 方向键逻辑 (如: 切换单手势编号、微调手腕位置等)
        // cmd->action = ACTION_...;
        break;

    default:
        break; // 未识别的按键，忽略
    }
}

/* ================================================================
 *  输入映射函数：CAN 消息 → ActionCmd
 * ================================================================ */

/**
 * @brief 将 CAN 总线消息映射为统一动作指令
 * @param msg  CAN 消息指针
 * @param cmd  [输出] 填充的动作指令
 *
 * CAN 协议约定 (ID = CAN_HAND_ID: 0x020 右手 或 0x021 左手):
 *   所有子命令为 16 位大端 (data[1]=高字节, data[2]=低字节)
 *
 *   data[0] = 0x00 → 心跳请求 (手回复 0x01)
 *
 *   data[0] = 0x10 → 手腕摆动控制
 *     子命令 (data[1..2]):
 *       0x0000=停, 0x0001=连续摆动,
 *       0x0002=最左, 0x0003=最右, 0x0004=回中,
 *       0x0005=设置周期 (data[3..4] = cycle_ms 大端 uint16)
 *
 *   data[0] = 0x11 → 手腕旋转控制
 *     子命令: 0x0000=停, 0x0001=连续旋转,
 *       0x0002-0x0004=预设位置, 0x0005=设置周期
 *
 *   data[0] = 0x12 → 手指连续动作 (固定速度)
 *     子命令 (data[1..2]): 0x0000=停, 0x0001=弯折, 0x0002=数数字, 0x0003=握拳
 *     data[3] = 循环次数 (8-bit)
 *
 *   data[0] = 0x13 → 手指单个动作
 *     子命令 (data[1..2]) = 手势编号
 *
 *   data[0] = 0x14 → 手指连续动作 (自定义速度)
 *     子命令 (data[1..2]): 0x0001=数数字, 0x0002=弯折, 0x0003=其他, 0x0004=握拳
 *     data[3..4] = 速度 ms (uint16 大端)
 *     data[5..6] = 循环次数 (uint16 大端)
 *
 *   以下为直接硬件指令，在 Task_CAN_Process_Entry 中处理，不经过本函数:
 *   data[0] = 0x15 → 全电机正转 (data[1..7]=关节01-07圈数)
 *   data[0] = 0x16 → 全电机反转 (data[1..7]=关节01-07圈数)
 *   data[0] = 0x17 → 编码器清零 (无附加数据)
 *   data[0] = 0x18 → 推杆电机定位 (data[1..4]=pos1H,pos1L,pos2H,pos2L)
 */
void CAN_Msg_MapToAction(const App_CAN_Msg_t *msg, ActionCmd_t *cmd)
{
    uint16_t subcmd = ((uint16_t)msg->data[1] << 8) | msg->data[2];  // 大端

    cmd->action = ACTION_NONE;
    cmd->param1 = 0;
    cmd->param2 = 0;

    if (msg->id != CAN_HAND_ID) return;

    switch (msg->data[0]) {

    case 0x10:   // 手腕摆动控制 (data[1..2]=16位子命令, 大端)
        switch (subcmd) {
        case 0x0000: cmd->action = ACTION_WRIST_PENDULUM_STOP;  break;
        case 0x0001: cmd->action = ACTION_WRIST_PENDULUM_START; break;
        case 0x0002: cmd->action = ACTION_WRIST_POS_PRESET1; cmd->param1 = 0x02; break;
        case 0x0003: cmd->action = ACTION_WRIST_POS_PRESET2; cmd->param1 = 0x03; break;
        case 0x0004: cmd->action = ACTION_WRIST_POS_PRESET3; cmd->param1 = 0x04; break;
        case 0x0005: // 设置周期: data[3..4] = cycle_ms (大端)
            cmd->action = ACTION_WRIST_SET_CYCLE;
            cmd->param1 = ((uint16_t)msg->data[3] << 8) | msg->data[4];
            break;
        }
        break;

    case 0x11:   // 手腕旋转控制 (data[1..2]=16位子命令, 大端)
        switch (subcmd) {
        case 0x0000: cmd->action = ACTION_WRIST_ROTATION_STOP;  break;
        case 0x0001: cmd->action = ACTION_WRIST_ROTATION_START; break;
        case 0x0002: cmd->action = ACTION_WRIST_POS_PRESET1; cmd->param1 = 0x02; break;
        case 0x0003: cmd->action = ACTION_WRIST_POS_PRESET2; cmd->param1 = 0x03; break;
        case 0x0004: cmd->action = ACTION_WRIST_POS_PRESET3; cmd->param1 = 0x04; break;
        case 0x0005: // 设置周期: data[3..4] = cycle_ms (大端)
            cmd->action = ACTION_WRIST_SET_CYCLE;
            cmd->param1 = ((uint16_t)msg->data[3] << 8) | msg->data[4];
            break;
        }
        break;

    case 0x12:   // 手指连续动作 (data[1..2]=类型, data[3]=次数, 固定速度)
        cmd->param1 = msg->data[3];  // 重复次数 (8-bit)
        switch (subcmd) {
        case 0x0000: cmd->action = ACTION_ALL_STOP;                            break;
        case 0x0001: cmd->action = ACTION_FINGER_BEND;  cmd->param2 = 300;  break;
        case 0x0002: cmd->action = ACTION_FINGER_COUNT; cmd->param2 = 1300; break;
        case 0x0003: cmd->action = ACTION_FINGER_FIST;  cmd->param2 = 1500; break;
        }
        break;

    case 0x13:   // 手指单个动作 (data[1..2]=16位手势编号, 大端)
        cmd->action = ACTION_FINGER_SINGLE;
        cmd->param1 = subcmd;
        break;

    case 0x14:   // 手指连续动作 (data[1..2]=类型, data[3..4]=速度ms, data[5..6]=次数)
        cmd->param1 = ((uint16_t)msg->data[5] << 8) | msg->data[6];  // 重复次数
        cmd->param2 = ((uint16_t)msg->data[3] << 8) | msg->data[4];  // 速度 (ms)
        switch (subcmd) {
        case 0x0001: cmd->action = ACTION_FINGER_COUNT;   break;
        case 0x0002: cmd->action = ACTION_FINGER_BEND;    break;
        case 0x0003: cmd->action = ACTION_FINGER_GESTURE; break;
        case 0x0004: cmd->action = ACTION_FINGER_FIST;    break;
        }
        break;

    }
}

/* ================================================================
 *  CAN 直接硬件指令 (不走 Action_Dispatch)
 *  0x15=全电机正转, 0x16=全电机反转, 0x17=编码器清零, 0x18=推杆定位
 * ================================================================ */

static void Can_AllMotors(const App_CAN_Msg_t *msg)
{
    // data[1..7] = 关节 01-07 圈数, 速度 0x64, 正转 (mode=1)
    Reg[4] = ((uint16_t)msg->data[1] << 8) | 0x64;
    Reg[5] = ((uint16_t)msg->data[2] << 8) | 0x64;
    Reg[6] = ((uint16_t)msg->data[3] << 8) | 0x64;
    Reg[7] = ((uint16_t)msg->data[7] << 8) | 0x64;
    Reg[8] = 0x0101;
    Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, &Reg[4]);
    osDelay(15);
    Reg[9]  = ((uint16_t)msg->data[4] << 8) | 0x64;
    Reg[10] = ((uint16_t)msg->data[5] << 8) | 0x64;
    Reg[11] = ((uint16_t)msg->data[6] << 8) | 0x64;
    Reg[12] = ((uint16_t)msg->data[7] << 8) | 0x64;
    Reg[13] = 0x0101;
    Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, &Reg[9]);
}

static void Can_AllMotorsRev(const App_CAN_Msg_t *msg)
{
    // 同 0x15, mode=3 反转
    Reg[4] = ((uint16_t)msg->data[1] << 8) | 0x64;
    Reg[5] = ((uint16_t)msg->data[2] << 8) | 0x64;
    Reg[6] = ((uint16_t)msg->data[3] << 8) | 0x64;
    Reg[7] = ((uint16_t)msg->data[7] << 8) | 0x64;
    Reg[8] = 0x0301;
    Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, &Reg[4]);
    osDelay(15);
    Reg[9]  = ((uint16_t)msg->data[4] << 8) | 0x64;
    Reg[10] = ((uint16_t)msg->data[5] << 8) | 0x64;
    Reg[11] = ((uint16_t)msg->data[6] << 8) | 0x64;
    Reg[12] = ((uint16_t)msg->data[7] << 8) | 0x64;
    Reg[13] = 0x0301;
    Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, &Reg[9]);
}

static void Can_ClearEncoders(void)
{
    Reg[4] = 0x0064; Reg[5] = 0x0064; Reg[6] = 0x0064; Reg[7] = 0x0000; Reg[8] = 0x0401;
    Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, &Reg[4]);
    osDelay(15);
    Reg[9] = 0x0064; Reg[10] = 0x0064; Reg[11] = 0x0064; Reg[12] = 0x0064; Reg[13] = 0x0401;
    Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, &Reg[9]);
}

static void Can_WristPos(const App_CAN_Msg_t *msg)
{
    // 0x18 pos1_H pos1_L pos2_H pos2_L, 速度 3500=0x0DAC
    vel_move(msg->data[1], msg->data[2], 0x0D, 0xAC);
    osDelay(1);
    vel_move1(msg->data[3], msg->data[4], 0x0D, 0xAC);
}

/* ================================================================
 *  手腕运动启停控制 (wrapper)
 *  所有调用方 (IR / CAN / Modbus / Action_Dispatch) 统一通过
 *  这两个函数向 Queue_WristCmd 发送消息，不再直接操作全局标志位。
 * ================================================================ */

void WristMotion_Start(uint8_t mode)
{
    WristCmd_t msg;
    if (mode == MODE_PENDULUM) {
        msg.cmd = WRIST_CMD_PENDULUM;
        msg.cycle_ms = cycle_time;
    } else if (mode == MODE_ROTATION) {
        msg.cmd = WRIST_CMD_ROTATION;
        msg.cycle_ms = cycle_time_R;
    } else {
        return;
    }
    osMessageQueuePut(Queue_WristCmd, &msg, 0, 0);
}

void WristMotion_Stop(void)
{
    WristCmd_t msg = { .cmd = WRIST_CMD_STOP, .cycle_ms = 0 };
    osMessageQueuePut(Queue_WristCmd, &msg, 0, 0);
}

/**
 * @brief 手腕运动 RTOS 任务 ( 消息队列驱动)
 *
 * 一条消息队列同时承载两个职能：
 *   1. 命令通道 — WristMotion_Start/Stop 向队列投递消息
 *   2. 心跳定时 — osMessageQueueGet(40ms 超时) 同时充当周期节拍
 *
 * stop 时没有消息入队，每次超时后判断 run_mode==STOP 即跳过，
 * 不阻塞、不轮询全局变量。
 */
void Task_WristMotion_Entry(void *argument)
{
    WristCmd_t msg;
    uint8_t    run_mode  = WRIST_CMD_STOP;  // 任务本地状态，非全局变量
    uint16_t   cycle_ms  = 2000;             // 默认周期，单位 ms

    for (;;)
    {
        // 阻塞等命令，超时 40ms → 作为运动心跳
        osStatus_t stat = osMessageQueueGet(Queue_WristCmd, &msg, NULL, 40);

        if (stat == osOK)   // 收到新命令
        {
            switch (msg.cmd) {
            case WRIST_CMD_PENDULUM:
            case WRIST_CMD_ROTATION:
                run_mode      = msg.cmd;
                cycle_ms      = msg.cycle_ms;
                motion_active = 0;               // 重置状态机，平滑起步
                break;

            case WRIST_CMD_STOP:
                run_mode      = WRIST_CMD_STOP;
                motion_active = 0;
                break;
            }
        }

        // 处于运行模式则执行一步运动
        if (run_mode != WRIST_CMD_STOP) {
            uint8_t wrist_mode = (run_mode == WRIST_CMD_PENDULUM)
                                 ? MODE_PENDULUM : MODE_ROTATION;
            updateSmartMotionControl(wrist_mode, cycle_ms);
        }
    }
}
void Task_Demo_Entry(void *argument){
    osDelay(1000); // 等待系统稳定
    vTaskDelete(NULL); 
    for (;;)
    {
        finger_wrist_show(); // 展示用的预设动作序列
        osDelay(5000);
        osDelay(5000);
        osDelay(3000);

    }
}
/* ================================================================
 *  初始化
 * ================================================================ */

// App_ModbusPC_StartReceive：启动USART1的DMA空闲帧接收并关闭半传输通知
// 参数：无
// 返回值：无，启动失败时由上位机任务周期重试
static void App_ModbusPC_StartReceive(void)
{
    taskENTER_CRITICAL();
    // 接收空闲与缓冲区可复用必须同时成立，防止超时唤醒恰逢IDLE交帧。
    if (modbus2.reflag == 0U && s_pc_rx_error == 0U &&
        huart1.RxState == HAL_UART_STATE_READY &&
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, modbus2.rcbuf,
                                    sizeof(modbus2.rcbuf)) == HAL_OK) {
        // 半缓冲到达不表示完整MODBUS帧，避免较长写入被提前解析。
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
    taskEXIT_CRITICAL();
}

// App_RequestSlaveDispatch：合并从板更新位并唤醒分发任务
// 参数：board_mask - bit0为一号板，bit1为二号板
// 返回值：无
void App_RequestSlaveDispatch(uint8_t board_mask)
{
    taskENTER_CRITICAL();
    s_slave_dispatch_mask |= board_mask & 0x03U;
    taskEXIT_CRITICAL();
    if (Sem_NewSlaveCmd != NULL) (void)osSemaphoreRelease(Sem_NewSlaveCmd);
}

// App_Comm_Init：建立控制、反馈和上报任务，分别启动USART1及两路CAN
// 参数：无，调度器启动前调用
// 返回值：无，软件资源建立失败时进入项目统一错误处理
void App_Comm_Init(void)
{
    /* 1. 创建信号量 */
    Sem_Uart1_RxIdle = osSemaphoreNew(1, 0, NULL);
    Sem_NewSlaveCmd  = osSemaphoreNew(1, 0, NULL);
    Sem_IR_Ready     = osSemaphoreNew(1, 0, NULL);
    Sem_NewMotionCmd = osSemaphoreNew(5, 0, NULL);  // 最多缓存 5 次打断信号

    /* 2. 创建消息队列 */
    Queue_CAN_Rx   = osMessageQueueNew(10, sizeof(App_CAN_Msg_t), NULL);
    Queue_WristCmd = osMessageQueueNew(4,  sizeof(WristCmd_t),  NULL); // 手腕命令
    if (!ModbusMaster_Init()) Error_Handler();

    /* 3. 创建任务 */
    const osThreadAttr_t modbus_attr = { .name = "ModbusPC", .stack_size = 512 * 4, .priority = osPriorityNormal };
    TaskModbusPCHandle = osThreadNew(Task_ModbusPC_Entry, NULL, &modbus_attr);

    const osThreadAttr_t dispatch_attr = { .name = "SlaveDispatch", .stack_size = 512 * 4, .priority = osPriorityBelowNormal };
    TaskSlaveDispatchHandle = osThreadNew(Task_SlaveDispatch_Entry, NULL, &dispatch_attr);

    const osThreadAttr_t can_attr = { .name = "CANProcess", .stack_size = 512 * 4, .priority = osPriorityNormal };
    TaskCANHandle = osThreadNew(Task_CAN_Process_Entry, NULL, &can_attr);

    const osThreadAttr_t ir_attr = { .name = "IRProcess", .stack_size = 512 * 4, .priority = osPriorityNormal };
    TaskIRHandle = osThreadNew(Task_IR_Process_Entry, NULL, &ir_attr);

    const osThreadAttr_t wrist_attr = { .name = "WristMotion", .stack_size = 512 * 4, .priority = osPriorityAboveNormal };
    osThreadNew(Task_WristMotion_Entry, NULL, &wrist_attr);

    //针对展示的灵巧手创建一个从上电就自动开始的任务，用于展示
    const osThreadAttr_t demo_attr = { .name = "DemoTask", .stack_size = 512 * 4, .priority = osPriorityAboveNormal };
    osThreadNew(Task_Demo_Entry, NULL, &demo_attr);

    /* 4. 启动硬件 */
    // USART1接收由其任务首次运行时启动，避免调度前进入RTOS临界区。

    // CAN 硬件过滤器: 仅接收 CAN_HAND_ID (右手 0x020 或左手 0x021，二选一)
    MyCan_Init();
    MyCan_ConfigFilter_List16Bit(0, CAN_HAND_ID, CAN_HAND_ID,
                                    CAN_HAND_ID, CAN_HAND_ID, CAN_RX_FIFO0);
    MyCan_Start();
    MyCan_SendStdData(CAN_HAND_ID, (uint8_t[]){CAN_HEARTBEAT_RESPONSE}, 1); // 上电即发心跳
    if (!SlaveFeedback_Init() || !Uart4Telemetry_Init()) Error_Handler();
    // 红外遥控初始化 (TIM3 输入捕获)
    InfraredRemote_Init();
}

/* ========================================================== */
/*                   RTOS 任务实现                             */
/* ========================================================== */

// Task_ModbusPC_Entry：解析USART1完整请求，处理接收错误并恢复DMA接收
// 参数：argument - 未使用
// 返回值：无，任务持续运行
void Task_ModbusPC_Entry(void *argument)
{
    (void)argument;
    App_ModbusPC_StartReceive();
    for(;;)
    {
        if (osSemaphoreAcquire(Sem_Uart1_RxIdle, 20U) == osOK) {
            if (s_pc_rx_error != 0U) {
                (void)HAL_UART_AbortReceive(&huart1);
                modbus2.reflag = 0U;
                modbus2.recount = 0U;
                s_pc_rx_error = 0U;
            } else {
                modbus2_Event();
            }
        }
        App_ModbusPC_StartReceive();
    }
}

// Task_SlaveDispatch_Entry：按更新掩码向对应从板下发寄存器快照
// 参数：argument - 未使用
// 返回值：无，任务持续运行
void Task_SlaveDispatch_Entry(void *argument)
{
    (void)argument;
    for(;;)
    {
        osSemaphoreAcquire(Sem_NewSlaveCmd, osWaitForever);
        uint16_t commands[10];
        taskENTER_CRITICAL();
        uint8_t board_mask = s_slave_dispatch_mask;
        s_slave_dispatch_mask = 0U;
        memcpy(commands, &Reg[4], sizeof(commands));
        taskEXIT_CRITICAL();
        if ((board_mask & 0x01U) != 0U) {
            Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, commands);
        }
        if ((board_mask & 0x03U) == 0x03U) osDelay(15);
        if ((board_mask & 0x02U) != 0U) {
            Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, &commands[5]);
        }
    }
}

/**
 * @brief 任务：处理 CAN 总线消息
 * @note  与红外完全解耦 — CAN 消息直接映射为 ActionCmd 并分发，
 *        不再像旧代码那样模拟红外信号 (rcvFlag + key_num)。
 */
void Task_CAN_Process_Entry(void *argument)
{
    App_CAN_Msg_t msg;
    ActionCmd_t cmd;
    uint8_t heartbeat_resp = CAN_HEARTBEAT_RESPONSE;

    for(;;)
    {
        if (osMessageQueueGet(Queue_CAN_Rx, &msg, NULL, osWaitForever) == osOK)
        {
            // ===================================================
            //  非动作消息: 心跳 / 直接硬件指令
            //  直接处理，不进入 MapToAction → Dispatch 流程
            // ===================================================

            // 心跳
            if (msg.data[0] == CAN_HEARTBEAT_REQUEST) {
                MyCan_SendStdData(CAN_HAND_ID, &heartbeat_resp, 1);
                continue;
            }

            // 直接硬件指令 (操作 Reg[] + 485 下发)
            if (msg.data[0] == 0x15) { Can_AllMotors(&msg);    continue; }
            if (msg.data[0] == 0x16) { Can_AllMotorsRev(&msg); continue; }
            if (msg.data[0] == 0x17) { Can_ClearEncoders();    continue; }
            if (msg.data[0] == 0x18) { Can_WristPos(&msg);     continue; }

            // ===================================================
            //  动作消息: 纯映射 → 纯分发
            // ===================================================
            CAN_Msg_MapToAction(&msg, &cmd);
            Action_Dispatch(&cmd);
        }
    }
}

/**
 * @brief 任务：处理红外遥控输入
 * @note  与 CAN 完全解耦 — IR 按键直接映射为 ActionCmd 并分发。
 */
void Task_IR_Process_Entry(void *argument)
{
    ActionCmd_t cmd;

    for(;;)
    {
        osSemaphoreAcquire(Sem_IR_Ready, osWaitForever);

        if (!sys_infrared_enable) continue;

        uint16_t key = InfraredRemote_Get_KeyNum();
        IR_Key_MapToAction(key, &cmd);
        Action_Dispatch(&cmd);
    }
}

/* ========================================================== */
/*                   HAL 硬件中断回调接管                       */
/* ========================================================== */

// HAL_UARTEx_RxEventCallback：只把USART1完整DMA事件交付给上位机任务
// 参数：huart - 触发串口；Size - 本次接收长度
// 返回值：无，半传输通知不作为完整请求
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1 && Sem_Uart1_RxIdle != NULL &&
        HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_HT &&
        Size > 0U && Size <= sizeof(modbus2.rcbuf))
    {
        modbus2.recount = (uint8_t)Size;
        modbus2.reflag = 1U;
        osSemaphoreRelease(Sem_Uart1_RxIdle);
    }
}

// HAL_UART_TxCpltCallback：只释放UART4反馈发送，保留其他串口控制链路
// 参数：huart - 完成发送的串口
// 返回值：无
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) Uart4Telemetry_OnTxComplete();
}

// HAL_UART_ErrorCallback：分离UART4发送异常和USART1接收恢复请求
// 参数：huart - 发生错误的串口
// 返回值：无，恢复操作交由任务执行
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        Uart4Telemetry_OnTxError();
    } else if (huart->Instance == USART1 && Sem_Uart1_RxIdle != NULL) {
        s_pc_rx_error = 1U;
        osSemaphoreRelease(Sem_Uart1_RxIdle);
    }
}

// HAL_CAN_RxFifo0MsgPendingCallback：按CAN实例分流并排空本次接收FIFO
// 参数：hcan - 触发接收的CAN外设
// 返回值：无；CAN2反馈绝不进入CAN1动作队列
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U) {
        App_CAN_Msg_t rxMsg = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxMsg.data) != HAL_OK) break;
        if (hcan->Instance == CAN2) {
            SlaveFeedback_OnCanRx(&rxHeader, rxMsg.data, HAL_GetTick());
        } else if (hcan->Instance == CAN1 && Queue_CAN_Rx != NULL &&
                   rxHeader.IDE == CAN_ID_STD && rxHeader.RTR == CAN_RTR_DATA &&
                   rxHeader.StdId == CAN_HAND_ID && rxHeader.DLC >= 1U && rxHeader.DLC <= 8U) {
            rxMsg.id = rxHeader.StdId;
            rxMsg.len = (uint8_t)rxHeader.DLC;
            osMessageQueuePut(Queue_CAN_Rx, &rxMsg, 0, 0);
        }
    }
}

