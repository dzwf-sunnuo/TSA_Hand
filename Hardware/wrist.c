#include "wrist.h"
#include "usart.h"
#include "math.h"

UART_Instruction_Frame fraMotor1; 
UART_Instruction_Frame fraMotor2;

static uint32_t calculateRequiredTime(float distance_mm, float max_speed_mm_s);
static uint16_t physicalToSpeed(float physical_speed);
static float speedToPhysical(uint16_t speed_encoded);
static void updatePendulumSmartMotion(uint32_t current_time);
static void updateRotationSmartMotion(uint32_t current_time);
static void updateRotationStatePosition(RotationState state);

// 全局变量
volatile uint8_t wrist_enabled = 0;           // 手腕运动使能标志
volatile uint8_t wrist_enabled_constand = 0;


// 运动控制参数
volatile WristMode current_mode = MODE_PENDULUM;  // 当前模式
volatile uint16_t current_speed = 0x07D0;         // 当前速度值(0-4000)
volatile uint8_t motion_active = 0;               // 运动是否激活

// 运动状态变量
static uint8_t motion_direction = 0;          // 运动方向 0:正向, 1:反向
static uint32_t motion_start_time = 0;        // 运动开始时间
static RotationState rotation_state = STATE_DOWN; // 旋转模式当前状态
static uint32_t state_start_time = 0;         // 状态开始时间

// 速度计算相关变量
static float current_physical_speed = 0.0f;   // 当前物理速度(mm/s)

uint16_t cycle_time = 3200;
uint16_t cycle_time_R = 3200;

// 计算校验和函数，除帧头外其余所有数据累加和的低八位
uint8_t calculate_checksum(uint8_t *data, uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

// 构建指令帧函数，参数根据实际需求传入
void build_instruction_frame(UART_Instruction_Frame *frame, 
                             uint8_t id_addr, uint8_t cmd, uint16_t reg_addr, 
                             uint8_t *data, uint8_t data_len) {
    if (data_len > MAX_FRAME_LENGTH) {
        return;
    }
    frame->header1 = 0x55;
    frame->header2 = 0xAA;
    frame->id_address = id_addr;
    frame->cmd_type = cmd;
    frame->register_addr = reg_addr;
    frame->data_length = data_len + 3; // L=N+3
    for (uint8_t i = 0; i < data_len; i++) {
        frame->data[i] = data[i];
    }
    // 计算校验和
    uint8_t checksum_data[MAX_FRAME_LENGTH];
    uint8_t checksum_index = 0;
    checksum_data[checksum_index++] = frame->data_length;
    checksum_data[checksum_index++] = frame->id_address;
    checksum_data[checksum_index++] = frame->cmd_type;
    checksum_data[checksum_index++] = (uint8_t)(frame->register_addr & 0xFF);
    checksum_data[checksum_index++] = (uint8_t)(frame->register_addr >> 8);
    for (uint8_t i = 0; i < data_len; i++) {
        checksum_data[checksum_index++] = frame->data[i];
    }
    frame->check_sum = calculate_checksum(checksum_data, checksum_index);
}
														 
// 发送指令帧函数，使用HAL库的串口发送函数
void send_uart_instruction_frame(UART_Instruction_Frame *frame, UART_HandleTypeDef *huart) {
    if (frame->data_length < 3 || frame->data_length - 3 > MAX_FRAME_LENGTH) {
        return;
    }
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
    HAL_UART_Transmit(huart, send_buffer, index, 1000); // 1000为超时时间，单位ms
}
//数据帧遵循小端存储，低位写在前面，高位写在后面，这里已经进行过处理，调用函数按正常写法就行
//速度控制
// 这里保留外部 API 名称，但它本质上只是在开环协议里下发速度命令。
void vel_move(uint8_t POS_H,uint8_t POS_L,uint8_t VEL_H,uint8_t VEL_L)
{
	  uint8_t data_buffer[10] = {0x02,0x00,0x00,0x00,0x00,0x00,VEL_L,VEL_H,POS_L,POS_H}; 
		build_instruction_frame(&fraMotor1, 0x01, 0x32, 0x0025, data_buffer, 0x10);
		send_uart_instruction_frame(&fraMotor1, &huart3);//电机1对应串口3
}
void vel_move1(uint8_t POS_H,uint8_t POS_L,uint8_t VEL_H,uint8_t VEL_L)
{
	  uint8_t data_buffer[10] = {0x02,0x00,0x00,0x00,0x00,0x00,VEL_L,VEL_H,POS_L,POS_H}; 
		build_instruction_frame(&fraMotor2, 0x01, 0x32, 0x0025, data_buffer, 0x10);
		send_uart_instruction_frame(&fraMotor2, &huart6);//电机2对应串口6
}


/**
 * @brief 将速度编码值转换为物理速度(mm/s)
 * @param speed_encoded 速度编码值(0-4000)
 * @return 物理速度(mm/s)
 */
static float speedToPhysical(uint16_t speed_encoded)
{
    return (float)speed_encoded / SPEED_RANGE * PHYSICAL_SPEED_RANGE;
}

/**
 * @brief 将物理速度转换为速度编码值
 * @param physical_speed 物理速度(mm/s)
 * @return 速度编码值(0-4000)
 */
static uint16_t physicalToSpeed(float physical_speed)
{
    if (physical_speed > PHYSICAL_SPEED_RANGE) {
        physical_speed = PHYSICAL_SPEED_RANGE;
    } else if (physical_speed < 0) {
        physical_speed = 0;
    }
    return (uint16_t)(physical_speed / PHYSICAL_SPEED_RANGE * SPEED_RANGE);
}

/**
 * @brief 根据速度和行程计算所需时间
 * @param distance_mm 需要移动的距离(mm)
 * @param max_speed_mm_s 最大速度(mm/s)
 * @return 所需时间(ms)
 */
static uint32_t calculateRequiredTime(float distance_mm, float max_speed_mm_s)
{
    // 考虑加速和减速阶段
    // 假设使用S曲线加速度，总时间 = 加速时间 + 匀速时间 + 减速时间
    
    // 简化计算：使用平均速度，并考虑加速减速时间
    // 加速度设为最大速度的1/3，这样加速和减速各需要3倍加速时间
    float acceleration_mm_s2 = max_speed_mm_s / 3.0f;
    
    // 加速到最大速度所需时间
    float acceleration_time_s = max_speed_mm_s / acceleration_mm_s2;
    
    // 加速和减速阶段移动的总距离
    float accel_decel_distance = max_speed_mm_s * acceleration_time_s; // s = v_avg * t = (v_max/2) * 2*t_accel
    
    if (distance_mm <= accel_decel_distance) {
        // 距离太短，无法加速到最大速度
        // 计算实际能达到的最大速度
        float actual_max_speed = sqrtf(acceleration_mm_s2 * distance_mm);
        float total_time_s = 2.0f * actual_max_speed / acceleration_mm_s2;
        return (uint32_t)(total_time_s * 1000.0f-(-0.15*cycle_time + 2450));
   //     return (uint32_t)(total_time_s * 1000.0f-(2200));

    } else {
        // 可以加速到最大速度
        float constant_distance = distance_mm - accel_decel_distance;
        float constant_time_s = constant_distance / max_speed_mm_s;
        float total_time_s = 2.0f * acceleration_time_s + constant_time_s;
        return (uint32_t)((total_time_s * 1000.0f)-(-0.15*cycle_time + 2450));
   //     return (uint32_t)((total_time_s * 1000.0f)-(2200));

    }
}

/**
 * @brief 智能速度与周期关联运动控制函数
 * @param mode 运动模式: 1=摆手模式, 2=旋转模式
 * @param speed_encoded 速度编码值(0-4000)
 * @note 这个函数需要在40ms定时器中断中调用
 */
void updateSmartMotionControl(uint8_t mode, uint16_t speed_encoded)
{
    uint32_t current_time = HAL_GetTick();
    
    // 更新模式参数
    if (mode == MODE_PENDULUM || mode == MODE_ROTATION)
    {
        current_mode = mode;
    }
    
    // 更新速度参数
    if (speed_encoded <= 0x0FA0)
    {
        current_speed = speed_encoded;
        current_physical_speed = speedToPhysical(current_speed);
    }
    
    // 如果运动未激活，初始化运动
    if (!motion_active)
    {
        motion_active = 1;
        motion_start_time = current_time;
        state_start_time = current_time;
        motion_direction = 0;  // 正向开始
        rotation_state = STATE_DOWN; // 从全下位置开始
        
        // 根据模式设置初始状态
        if (current_mode == MODE_PENDULUM)
        {
            // 摆手模式：初始方向为从左到右
            // 目标位置固定为右位置
            vel_move((POS_PENDULUM_RIGHT_1 >> 8) & 0xFF, POS_PENDULUM_RIGHT_1 & 0xFF,
                     (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            vel_move1((POS_PENDULUM_RIGHT_2 >> 8) & 0xFF, POS_PENDULUM_RIGHT_2 & 0xFF,
                      (current_speed >> 8) & 0xFF, current_speed & 0xFF);
        }
        else if (current_mode == MODE_ROTATION)
        {
            // 旋转模式：初始状态为全下，下一个目标是手腕左
            // 目标位置固定为手腕左位置
            vel_move((POS_ROT_LEFT_1 >> 8) & 0xFF, POS_ROT_LEFT_1 & 0xFF,
                     (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            vel_move1((POS_ROT_LEFT_2 >> 8) & 0xFF, POS_ROT_LEFT_2 & 0xFF,
                      (current_speed >> 8) & 0xFF, current_speed & 0xFF);
        }
        return;
    }
    
    // 根据模式执行不同的运动
    if (current_mode == MODE_PENDULUM)
    {
        // 模式1：摆手运动
        updatePendulumSmartMotion(current_time);
    }
    else if (current_mode == MODE_ROTATION)
    {
        // 模式2：转动手腕
        updateRotationSmartMotion(current_time);
    }
}

/**
 * @brief 智能摆手运动控制
 * @param current_time 当前时间(ms)
 */
static void updatePendulumSmartMotion(uint32_t current_time)
{
    // 计算单程需要移动的距离(16mm)
    float single_distance_mm = PHYSICAL_POS_RANGE;
    
    // 计算单程所需时间
    uint32_t single_way_time = calculateRequiredTime(single_distance_mm, current_physical_speed);
    
    // 添加一点缓冲时间(如20ms)以确保平稳切换
    single_way_time += 20;
    
    // 计算总运动时间（一个来回）
    uint32_t elapsed_time = current_time - motion_start_time;
    uint32_t half_cycle_time = single_way_time;
    
    // 检查是否需要切换方向
    if (elapsed_time >= half_cycle_time)
    {
        // 切换方向
        motion_direction = !motion_direction;
        motion_start_time = current_time;
        elapsed_time = 0;
        
        // 根据新方向设置目标位置
        if (motion_direction == 0)
        {
            // 从左到右：目标位置固定为右位置
            vel_move((POS_PENDULUM_RIGHT_1 >> 8) & 0xFF, POS_PENDULUM_RIGHT_1 & 0xFF,
                     (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            vel_move1((POS_PENDULUM_RIGHT_2 >> 8) & 0xFF, POS_PENDULUM_RIGHT_2 & 0xFF,
                      (current_speed >> 8) & 0xFF, current_speed & 0xFF);
        }
        else
        {
            // 从右到左：目标位置固定为左位置
            vel_move((POS_PENDULUM_LEFT_1 >> 8) & 0xFF, POS_PENDULUM_LEFT_1 & 0xFF,
                     (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            vel_move1((POS_PENDULUM_LEFT_2 >> 8) & 0xFF, POS_PENDULUM_LEFT_2 & 0xFF,
                      (current_speed >> 8) & 0xFF, current_speed & 0xFF);
        }
        return;
    }
    
    // 计算当前阶段的速度曲线（S曲线）
    float progress = (float)elapsed_time / half_cycle_time;
    float speed_factor;
    
    // 使用正弦函数实现S曲线速度控制
    if (progress < 0.5f)
    {
        // 加速阶段
        speed_factor = sinf(M_PI * progress);
    }
    else
    {
        // 减速阶段
        speed_factor = sinf(M_PI * (1.0f - progress));
    }
    
    // 计算当前速度
    uint16_t instant_speed = physicalToSpeed(current_physical_speed * speed_factor);
    
    // 限制最小速度（防止速度为0）
    if (instant_speed < 0x03E8) instant_speed = 0x03E8;
    
    // 发送速度命令（位置保持不变）
    if (motion_direction == 0)
    {
        // 从左到右
        vel_move((POS_PENDULUM_RIGHT_1 >> 8) & 0xFF, POS_PENDULUM_RIGHT_1 & 0xFF,
                 (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
        vel_move1((POS_PENDULUM_RIGHT_2 >> 8) & 0xFF, POS_PENDULUM_RIGHT_2 & 0xFF,
                  (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
    }
    else
    {
        // 从右到左
        vel_move((POS_PENDULUM_LEFT_1 >> 8) & 0xFF, POS_PENDULUM_LEFT_1 & 0xFF,
                 (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
        vel_move1((POS_PENDULUM_LEFT_2 >> 8) & 0xFF, POS_PENDULUM_LEFT_2 & 0xFF,
                  (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
    }
}

/**
 * @brief 智能旋转运动控制
 * @param current_time 当前时间(ms)
 */
static void updateRotationSmartMotion(uint32_t current_time)
{
    // 计算每个状态需要移动的距离(16mm)
    float state_distance_mm = PHYSICAL_POS_RANGE;
    
    // 计算每个状态所需时间
    uint32_t state_duration = calculateRequiredTime(state_distance_mm, current_physical_speed) - (cycle_time_R*0.07);
    
    // 添加缓冲时间(如30ms)以确保平稳切换
//    state_duration += 30;
    
    // 计算在当前状态的持续时间
    uint32_t elapsed_state_time = current_time - state_start_time;
    
    // 检查是否需要切换到下一个状态
    if (elapsed_state_time >= state_duration)
    {
        // 切换到下一个状态
        rotation_state = (RotationState)((rotation_state + 1) % 4);
        state_start_time = current_time;
        elapsed_state_time = 0;
        
        // 根据新状态设置目标位置
        updateRotationStatePosition(rotation_state);
        return;
    }
    
    // 计算当前状态内的速度曲线
    float progress = (float)elapsed_state_time / state_duration;
    float speed_factor;
    
    // 三段式速度控制：加速-匀速-减速
    if (progress < 0.2f)
    {
        // 加速阶段(前20%)
        speed_factor = 0.3f + 0.7f * (progress / 0.2f);
    }
    else if (progress < 0.8f)
    {
        // 匀速阶段(中间60%)
        speed_factor = 1.0f;
    }
    else
    {
        // 减速阶段(后20%)
        speed_factor = 1.0f - 0.7f * ((progress - 0.8f) / 0.2f);
    }
    
    // 计算当前速度
    uint16_t instant_speed = physicalToSpeed(current_physical_speed * speed_factor);
    
    // 限制最小速度
    if (instant_speed < 0x03E8) instant_speed = 0x03E8;
    
    // 发送速度命令（位置根据当前状态确定）
    switch (rotation_state)
    {
        case STATE_DOWN:
            vel_move((POS_ROT_DOWN_1 >> 8) & 0xFF, POS_ROT_DOWN_1 & 0xFF,
                     (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            vel_move1((POS_ROT_DOWN_2 >> 8) & 0xFF, POS_ROT_DOWN_2 & 0xFF,
                      (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            break;
            
        case STATE_LEFT:
            vel_move((POS_ROT_LEFT_1 >> 8) & 0xFF, POS_ROT_LEFT_1 & 0xFF,
                     (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            vel_move1((POS_ROT_LEFT_2 >> 8) & 0xFF, POS_ROT_LEFT_2 & 0xFF,
                      (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            break;
            
        case STATE_UP:
            vel_move((POS_ROT_UP_1 >> 8) & 0xFF, POS_ROT_UP_1 & 0xFF,
                     (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            vel_move1((POS_ROT_UP_2 >> 8) & 0xFF, POS_ROT_UP_2 & 0xFF,
                      (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            break;
            
        case STATE_RIGHT:
            vel_move((POS_ROT_RIGHT_1 >> 8) & 0xFF, POS_ROT_RIGHT_1 & 0xFF,
                     (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            vel_move1((POS_ROT_RIGHT_2 >> 8) & 0xFF, POS_ROT_RIGHT_2 & 0xFF,
                      (instant_speed >> 8) & 0xFF, instant_speed & 0xFF);
            break;
    }
}

/**
 * @brief 更新旋转状态的目标位置
 * @param state 目标状态
 */
static void updateRotationStatePosition(RotationState state)
{
    switch (state)
    {
        case STATE_DOWN:
            // 下一个目标是手腕左
            vel_move((POS_ROT_LEFT_1 >> 8) & 0xFF, POS_ROT_LEFT_1 & 0xFF,
                     (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            vel_move1((POS_ROT_LEFT_2 >> 8) & 0xFF, POS_ROT_LEFT_2 & 0xFF,
                      (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            break;
            
        case STATE_LEFT:
            // 下一个目标是全上
            vel_move((POS_ROT_UP_1 >> 8) & 0xFF, POS_ROT_UP_1 & 0xFF,
                     (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            vel_move1((POS_ROT_UP_2 >> 8) & 0xFF, POS_ROT_UP_2 & 0xFF,
                      (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            break;
            
        case STATE_UP:
            // 下一个目标是手腕右
            vel_move((POS_ROT_RIGHT_1 >> 8) & 0xFF, POS_ROT_RIGHT_1 & 0xFF,
                     (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            vel_move1((POS_ROT_RIGHT_2 >> 8) & 0xFF, POS_ROT_RIGHT_2 & 0xFF,
                      (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            break;
            
        case STATE_RIGHT:
            // 下一个目标是全下
            vel_move((POS_ROT_DOWN_1 >> 8) & 0xFF, POS_ROT_DOWN_1 & 0xFF,
                     (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            vel_move1((POS_ROT_DOWN_2 >> 8) & 0xFF, POS_ROT_DOWN_2 & 0xFF,
                      (current_speed >> 8) & 0xFF, current_speed & 0xFF);
            break;
    }
}

/**
 * @brief 设置运动速度并重新计算周期
 * @param speed_encoded 速度编码值(0-4000)
 */
void setSmartMotionSpeed(uint16_t speed_encoded)
{
    if (speed_encoded <= 0x0FA0)
    {
        current_speed = speed_encoded;
        current_physical_speed = speedToPhysical(current_speed);
        
        // 重置运动，让新的速度立即生效
        motion_active = 0;
    }
}

/**
 * @brief 设置运动模式
 * @param mode 运动模式: 1=摆手, 2=旋转
 */
void setSmartMotionMode(uint8_t mode)
{
    if (mode == MODE_PENDULUM || mode == MODE_ROTATION)
    {
        current_mode = mode;
        motion_active = 0;  // 重置运动状态
    }
}

/**
 * @brief 启动/停止运动
 * @param enable 1=启动, 0=停止
 */
void enableSmartMotion(uint8_t enable)
{
    if (enable)
    {
        motion_active = 0;  // 重置状态，下次调用会重新初始化
    }
    else
    {
        motion_active = 0;
        // 发送停止命令（速度设为0，位置保持当前）
        vel_move(0, 0, 0, 0);
        vel_move1(0, 0, 0, 0);
    }
}

/**
 * @brief 计算并显示当前运动周期
 * @param mode 运动模式
 * @param speed_encoded 速度编码值
 * @return 预计周期时间(ms)
 */
void set_wrist_motion_Can(uint8_t data0,uint8_t data1)
{
	if(data0 == 0x10){    //左右运动
    switch (data1)
    {
		case 0x00: wrist_enabled = 0;wrist_enabled_constand = 0;vel_move1(0x04,0x00,0x0B,0xF0); vel_move(0x04,0x00,0x0B,0xF0);break; //失能
		case 0x01: wrist_enabled = 1;wrist_enabled_constand = 0;break; //连续左右摆手
		case 0x02: vel_move (0x00,0x2C,0x0B,0xF0); vel_move1(0x07,0x00,0x0B,0xF0); break; //最左
		case 0x03: vel_move1(0x00,0x2C,0x0B,0xF0); vel_move (0x07,0x00,0x0B,0xF0); break; //最右
		case 0x04: vel_move1(0x04,0x00,0x0B,0xF0); vel_move (0x04,0x00,0x0B,0xF0); break; //水平
	}
}

		else if(data0 == 0x11){    //前后运动
    switch (data1)
    {
		case 0x00: wrist_enabled = 0;wrist_enabled_constand = 0;vel_move1(0x04,0x00,0x0B,0xF0); vel_move(0x04,0x00,0x0B,0xF0);break; //失能
		case 0x01: wrist_enabled = 0;wrist_enabled_constand = 1;break; //连续转动手腕
		case 0x02: vel_move (0x07,0x0D,0x0B,0xF0); vel_move1(0x07,0x0D,0x0B,0xF0); break; //最上
		case 0x03: vel_move1(0x00,0x64,0x0B,0xF0); vel_move (0x00,0x64,0x0B,0xF0); break; //最下
		case 0x04: vel_move1(0x04,0x00,0x0B,0xF0); vel_move (0x04,0x00,0x0B,0xF0); break; //水平
	}
}


}
