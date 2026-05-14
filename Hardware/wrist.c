#include "wrist.h"
#include "usart.h"
#include "math.h"

UART_Instruction_Frame fraMotor1; 
UART_Instruction_Frame fraMotor2;

/* 全局控制变量 */
volatile uint8_t wrist_enabled = 0;           // 摆手运动使能
volatile uint8_t wrist_enabled_constand = 0;  // 旋转运动使能

volatile WristMode current_mode = MODE_PENDULUM;
volatile uint8_t motion_active = 0;

/* 运动参数变量 */
volatile uint16_t cycle_time = 3200;     // 摆动一个来回的时间(ms)
volatile uint16_t cycle_time_R = 3200;   // 旋转一个来回的时间(ms)


/* 运动状态机变量 */
static uint8_t motion_direction = 0;          // 0:向右, 1:向左
static uint32_t motion_step = 0;              // 定时器触发步数计数
static RotationState rotation_state = STATE_DOWN; 

/* 定时器中断周期 (ms) - 与htim2设置的40ms对应 */
#define TIM2_PERIOD_MS 40

/* 内部函数声明 */
static void updatePendulumSmartMotion(uint16_t cycle_time_ms);
static void updateRotationSmartMotion(uint16_t cycle_time_ms);

/**
 * @brief 计算校验和
 */
uint8_t calculate_checksum(uint8_t *data, uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/**
 * @brief 构建UART指令帧
 */
void build_instruction_frame(UART_Instruction_Frame *frame, 
                             uint8_t id_addr, uint8_t cmd, uint16_t reg_addr, 
                             uint8_t *data, uint8_t data_len) {
    if (data_len > MAX_FRAME_LENGTH) return;
    
    frame->header1 = 0x55;
    frame->header2 = 0xAA;
    frame->id_address = id_addr;
    frame->cmd_type = cmd;
    frame->register_addr = reg_addr;
    frame->data_length = data_len + 3;
    
    for (uint8_t i = 0; i < data_len; i++) {
        frame->data[i] = data[i];
    }
    
    uint8_t checksum_data[MAX_FRAME_LENGTH];
    uint8_t idx = 0;
    checksum_data[idx++] = frame->data_length;
    checksum_data[idx++] = frame->id_address;
    checksum_data[idx++] = frame->cmd_type;
    checksum_data[idx++] = (uint8_t)(frame->register_addr & 0xFF);
    checksum_data[idx++] = (uint8_t)(frame->register_addr >> 8);
    for (uint8_t i = 0; i < data_len; i++) {
        checksum_data[idx++] = frame->data[i];
    }
    frame->check_sum = calculate_checksum(checksum_data, idx);
}
                                                         
/**
 * @brief 发送UART指令帧 (非阻塞/极短超时，防止卡死定时器中断)
 */
void send_uart_instruction_frame(UART_Instruction_Frame *frame, UART_HandleTypeDef *huart) {
    if (frame->data_length < 3 || frame->data_length - 3 > MAX_FRAME_LENGTH) return;
    
    uint8_t send_buffer[MAX_FRAME_LENGTH];
    uint8_t index = 0;
    send_buffer[index++] = frame->header1;
    send_buffer[index++] = frame->header2;
    send_buffer[index++] = frame->data_length;
    send_buffer[index++] = frame->id_address;
    send_buffer[index++] = frame->cmd_type;
    send_buffer[index++] = (uint8_t)(frame->register_addr & 0xFF);
    send_buffer[index++] = (uint8_t)(frame->register_addr >> 8);
    for (uint8_t i = 0; i < frame->data_length - 3; i++) {
        send_buffer[index++] = frame->data[i];
    }
    send_buffer[index++] = frame->check_sum;
    
    /* 波特率115200下发11字节只需不足1ms，设置超时时间为2ms，防止通讯阻塞定时器中断 */
    HAL_UART_Transmit(huart, send_buffer, index, 2); 
}

/**
 * @brief 控制手腕电机1 (对应串口3)
 */
void vel_move(uint8_t POS_H,uint8_t POS_L,uint8_t VEL_H,uint8_t VEL_L)
{
    uint8_t data_buffer[10] = {0x02,0x00,0x00,0x00,0x00,0x00,VEL_L,VEL_H,POS_L,POS_H}; 
    build_instruction_frame(&fraMotor1, 0x01, 0x32, 0x0025, data_buffer, 0x10);
    send_uart_instruction_frame(&fraMotor1, &huart3);
}

/**
 * @brief 控制手腕电机2 (对应串口6)
 */
void vel_move1(uint8_t POS_H,uint8_t POS_L,uint8_t VEL_H,uint8_t VEL_L)
{
    uint8_t data_buffer[10] = {0x02,0x00,0x00,0x00,0x00,0x00,VEL_L,VEL_H,POS_L,POS_H}; 
    build_instruction_frame(&fraMotor2, 0x01, 0x32, 0x0025, data_buffer, 0x10);
    send_uart_instruction_frame(&fraMotor2, &huart6);
}

/**
 * @brief 智能运动控制核心调度
 * @param mode 模式: 1=摆动, 2=旋转
 * @param cycle_time_ms 运动一个来回的总时间(ms)
 * @note  由 TIM2 中断(40ms)调用
 */
void updateSmartMotionControl(uint8_t mode, uint16_t cycle_time_ms)
{
    if (mode == MODE_PENDULUM || mode == MODE_ROTATION) {
        current_mode = mode;
    }
    
    /* 初始化运动状态 */
    if (!motion_active) {
        motion_active = 1;
        motion_step = 0;
        motion_direction = 0;  
        rotation_state = STATE_DOWN; 
        
        /* 为了防止起始突变，初始速度给定最低速度，目标位置先给定一侧端点 */
        if (current_mode == MODE_PENDULUM) {
            vel_move((POS_PENDULUM_RIGHT_1 >> 8) & 0xFF, POS_PENDULUM_RIGHT_1 & 0xFF, (MIN_SPEED >> 8) & 0xFF, MIN_SPEED & 0xFF);
            vel_move1((POS_PENDULUM_RIGHT_2 >> 8) & 0xFF, POS_PENDULUM_RIGHT_2 & 0xFF, (MIN_SPEED >> 8) & 0xFF, MIN_SPEED & 0xFF);
        }
        else if (current_mode == MODE_ROTATION) {
            vel_move((POS_ROT_LEFT_1 >> 8) & 0xFF, POS_ROT_LEFT_1 & 0xFF, (MIN_SPEED >> 8) & 0xFF, MIN_SPEED & 0xFF);
            vel_move1((POS_ROT_LEFT_2 >> 8) & 0xFF, POS_ROT_LEFT_2 & 0xFF, (MIN_SPEED >> 8) & 0xFF, MIN_SPEED & 0xFF);
        }
        return;
    }
    
    /* 分发至具体的运动学算法 */
    if (current_mode == MODE_PENDULUM) {
        updatePendulumSmartMotion(cycle_time_ms);
    }
    else if (current_mode == MODE_ROTATION) {
        updateRotationSmartMotion(cycle_time_ms);
    }
}

/**
 * @brief 左右摆动手腕控制 (优化后的步进式绝对平滑 S 曲线)
 * @param cycle_time_ms 一个来回(左右各一次)的总时间
 */
static void updatePendulumSmartMotion(uint16_t cycle_time_ms)
{
    /* 一个来回是 cycle_time，单程(半周期)是 cycle_time / 2 */
    uint32_t half_cycle_ms = cycle_time_ms / 2;
    if (half_cycle_ms < 100) half_cycle_ms = 100; // 极值保护
    
    /* 计算单程需要的总步数 (每步对应 TIM2_PERIOD_MS 即 40ms) */
    uint32_t total_steps = half_cycle_ms / TIM2_PERIOD_MS;
    if (total_steps == 0) total_steps = 1;
    
    /* 检查是否达到单程终点，需要切换方向 */
    if (motion_step >= total_steps) {
        motion_direction = !motion_direction;
        motion_step = 0;
    }
    
    /* 计算当前周期内的进度 (0.0 ~ 1.0) */
    float progress = (float)motion_step / (float)total_steps;
    
    /* 采用正弦半波产生 S 曲线速度因子 sin(pi * progress)，两端速度趋于0，中间速度最大 */
    float speed_factor = sinf(M_PI * progress);
    
    /* 
     * 运动学数学推导:
     * 如果速度曲线为 v(t) = V_max * sin(pi * t / T)
     * 那么在这段时间 T 内的位移积分 S = V_max * T / (pi / 2)
     * 所以 V_max = S * pi / (2 * T)
     * 
     * 这里的寄存器距离 S 是 2000 步 (0x07D0)。
     */
    float T_s = (float)half_cycle_ms / 1000.0f;           // 单程时间(秒)
    float V_max = 2000.0f * M_PI / (2.0f * T_s);          // 推导出的峰值速度
    
    /* 计算当前瞬时速度并限幅 */
    uint16_t instant_speed = (uint16_t)(V_max * speed_factor);
    
    /* 
     * 防止死区：若速度极低，电机可能因内部摩擦不动作而产生滞后
     * 给定 MIN_SPEED (例如 100) 保证电机的连续响应 
     */
    if (instant_speed < MIN_SPEED) instant_speed = MIN_SPEED; 
    if (instant_speed > MAX_SPEED) instant_speed = MAX_SPEED;
    
    /* 将当前瞬时速度下发给目标位置 */
    if (motion_direction == 0)
    {
        /* 目标：向右 */
        vel_move((POS_PENDULUM_RIGHT_1 >> 8) & 0xFF, POS_PENDULUM_RIGHT_1 & 0xFF,
                 (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
        vel_move1((POS_PENDULUM_RIGHT_2 >> 8) & 0xFF, POS_PENDULUM_RIGHT_2 & 0xFF,
                  (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
    }
    else
    {
        /* 目标：向左 */
        vel_move((POS_PENDULUM_LEFT_1 >> 8) & 0xFF, POS_PENDULUM_LEFT_1 & 0xFF,
                 (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
        vel_move1((POS_PENDULUM_LEFT_2 >> 8) & 0xFF, POS_PENDULUM_LEFT_2 & 0xFF,
                  (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
    }
    
    motion_step++; // 增加步数，等待下个 40ms 中断
}

/**
 * @brief 上下左右旋转手腕控制 (步进式梯形速度曲线)
 */
static void updateRotationSmartMotion(uint16_t cycle_time_ms)
{
    /* 旋转分为4个状态(下、左、上、右)，每个状态耗时 1/4 周期 */
    uint32_t state_duration_ms = cycle_time_ms / 4;
    if (state_duration_ms < 100) state_duration_ms = 100;
    
    uint32_t total_steps = state_duration_ms / TIM2_PERIOD_MS;
    if (total_steps == 0) total_steps = 1;
    
    /* 检查是否需要切换至下一个状态 */
    if (motion_step >= total_steps) {
        rotation_state = (RotationState)((rotation_state + 1) % 4);
        motion_step = 0;
    }
    
    float progress = (float)motion_step / (float)total_steps;
    float speed_factor;
    
    /* 
     * 梯形速度曲线: 前20%加速，中间60%匀速，后20%减速 
     */
    if (progress < 0.2f) {
        speed_factor = 0.3f + 0.7f * (progress / 0.2f);
    } else if (progress < 0.8f) {
        speed_factor = 1.0f;
    } else {
        speed_factor = 1.0f - 0.7f * ((progress - 0.8f) / 0.2f);
    }
    
    /* 距离近似 2000 步，乘以 1.2 系数补偿加减速阶段带来的位移损失 */
    float T_s = (float)state_duration_ms / 1000.0f;
    float V_max = (2000.0f / T_s) * 1.2f; 
    
    uint16_t instant_speed = (uint16_t)(V_max * speed_factor);
    if (instant_speed < MIN_SPEED) instant_speed = MIN_SPEED;
    if (instant_speed > MAX_SPEED) instant_speed = MAX_SPEED;
    
    switch (rotation_state)
    {
        case STATE_DOWN:
            vel_move((POS_ROT_LEFT_1 >> 8) & 0xFF, POS_ROT_LEFT_1 & 0xFF,
                     (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            vel_move1((POS_ROT_LEFT_2 >> 8) & 0xFF, POS_ROT_LEFT_2 & 0xFF,
                      (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            break;
            
        case STATE_LEFT:
            vel_move((POS_ROT_UP_1 >> 8) & 0xFF, POS_ROT_UP_1 & 0xFF,
                     (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            vel_move1((POS_ROT_UP_2 >> 8) & 0xFF, POS_ROT_UP_2 & 0xFF,
                      (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            break;
            
        case STATE_UP:
            vel_move((POS_ROT_RIGHT_1 >> 8) & 0xFF, POS_ROT_RIGHT_1 & 0xFF,
                     (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            vel_move1((POS_ROT_RIGHT_2 >> 8) & 0xFF, POS_ROT_RIGHT_2 & 0xFF,
                      (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            break;
            
        case STATE_RIGHT:
            vel_move((POS_ROT_DOWN_1 >> 8) & 0xFF, POS_ROT_DOWN_1 & 0xFF,
                     (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            vel_move1((POS_ROT_DOWN_2 >> 8) & 0xFF, POS_ROT_DOWN_2 & 0xFF,
                      (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            break;
    }
    
    motion_step++;
}

/**
 * @brief 处理CAN总线下发的手腕控制指令
 */
void set_wrist_motion_Can(uint8_t data0,uint8_t data1)
{
    if(data0 == 0x10) // 摆动运动控制
    {    
        switch (data1)
        {
            case 0x00: // 停止
                wrist_enabled = 0;
                wrist_enabled_constand = 0;
                motion_active = 0;
                vel_move1(0x04,0x00,0x0B,0xF0); 
                vel_move(0x04,0x00,0x0B,0xF0);
                break; 
            case 0x01: // 开启摆动
                wrist_enabled = 1;
                wrist_enabled_constand = 0;
                motion_active = 0; // 重置运动状态，实现平滑起步
                break; 
            // 预设的固定位置动作
            case 0x02: vel_move (0x00,0x2C,0x0B,0xF0); vel_move1(0x07,0x00,0x0B,0xF0); break; 
            case 0x03: vel_move1(0x00,0x2C,0x0B,0xF0); vel_move (0x07,0x00,0x0B,0xF0); break; 
            case 0x04: vel_move1(0x04,0x00,0x0B,0xF0); vel_move (0x04,0x00,0x0B,0xF0); break; 
        }
    }
    else if(data0 == 0x11) // 旋转运动控制
    {    
        switch (data1)
        {
            case 0x00: // 停止
                wrist_enabled = 0;
                wrist_enabled_constand = 0;
                motion_active = 0;
                vel_move1(0x04,0x00,0x0B,0xF0); 
                vel_move(0x04,0x00,0x0B,0xF0);
                break; 
            case 0x01: // 开启旋转
                wrist_enabled = 0;
                wrist_enabled_constand = 1;
                motion_active = 0; // 重置运动状态
                break; 
            // 预设的固定位置动作
            case 0x02: vel_move (0x07,0x0D,0x0B,0xF0); vel_move1(0x07,0x0D,0x0B,0xF0); break; 
            case 0x03: vel_move1(0x00,0x64,0x0B,0xF0); vel_move (0x00,0x64,0x0B,0xF0); break; 
            case 0x04: vel_move1(0x04,0x00,0x0B,0xF0); vel_move (0x04,0x00,0x0B,0xF0); break; 
        }
    }
}
