#include "power_loss.h"
#include "Motor.h"
#include "stm32f4xx_hal.h"

// ================================================================
//  12V 掉电检测参数
// ================================================================
// 分压比: 10K/(100K+10K) = 1/11
// 12V → ADC = 12 × 1/11 / 3.3 × 4096 ≈ 1352
// 阈值取 ~6.5V 对应位置 (685), 留足余量防止误触发
#define PL_12V_THRESHOLD      700    // ADC counts, 对应 ~6.5V 输入
#define PL_DEBOUNCE_TICKS     2      // 连续 2 次采样一致才切换 (100ms 周期 → 200ms)

extern ADC_HandleTypeDef hadc2;      // CubeMX 生成, 需重新生成代码

static int pl_12v_state     = 1;     // 当前状态: 1=有电, 0=掉电
static int pl_debounce_cnt  = 0;

// ================================================================
//  初始化
// ================================================================
void PL_Init(void)
{
    // adc2 已由 CubeMX 在 MX_ADC2_Init() 中初始化
    pl_12v_state    = 1;
    pl_debounce_cnt = 0;

    // 上电默认开 LED (PE2 低电平有效, LED=ON 表示 12V 在线)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
}

// ================================================================
//  12V 检测 
// ================================================================
int PL_Is12VAlive(void)
{
    // 单次采样 ADC2 IN4 (轮询, ~5μs)
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 1) != HAL_OK) {
        HAL_ADC_Stop(&hadc2);
        return pl_12v_state;  // 采样失败 → 保持原状态, 避免误判
    }
    uint32_t val = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    int raw_alive = (val > PL_12V_THRESHOLD) ? 1 : 0;

    // 消抖: 连续 N 次一致才切换状态
    if (raw_alive != pl_12v_state) {
        if (++pl_debounce_cnt >= PL_DEBOUNCE_TICKS) {
            pl_12v_state    = raw_alive;
            pl_debounce_cnt = 0;
        }
    } else {
        pl_debounce_cnt = 0;
    }

    return pl_12v_state;
}

// ================================================================
//  紧急停机 (掉电时: 关闭 PWM 输出, 保留编码器, 关 LED)
//  直接停 TIM1/TIM5 的 PWM 通道, 断电后即使编码器转动也不生成 PWM
// ================================================================
void PL_EmergencyStop(void)
{
    for (int i = 0; i < Motor_Num; i++) {
        HAL_TIM_PWM_Stop(motor_hw_map[i].pwm_tim, motor_hw_map[i].pwm_ch1);
        HAL_TIM_PWM_Stop(motor_hw_map[i].pwm_tim, motor_hw_map[i].pwm_ch2);
    }
    // 关 LED 省电池 (PE2 低电平有效)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
}

// ================================================================
//  恢复运行 (12V 恢复时: 重启 PWM + 开 LED)
//  Motor_Control_Loop 在下个 10ms 周期自动恢复 PID 输出
// ================================================================
void PL_Resume(void)
{
    for (int i = 0; i < Motor_Num; i++) {
        HAL_TIM_PWM_Start(motor_hw_map[i].pwm_tim, motor_hw_map[i].pwm_ch1);
        HAL_TIM_PWM_Start(motor_hw_map[i].pwm_tim, motor_hw_map[i].pwm_ch2);
    }
    // 开 LED (PE2 低电平有效)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
}
