#include "Infrared1838.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "string.h"
#include "cmsis_os.h"

/* 由 app_comm_tasks.c 提供的 IR 信号量 (中断中释放，唤醒 IR 任务) */
extern osSemaphoreId_t Sem_IR_Ready;

uint32_t valueUp;        // 捕获到上升沿时刻的值
uint32_t valueDown;      // 捕获到下降沿时刻的值
uint8_t  isUpCapture = 1;// 判断是否为上升沿捕获
uint16_t PluseWidth;     // 脉宽（上升沿和下降沿捕获时间差）
static uint16_t frameBuffer[33] = {0}; // 一帧数据快照：同步码1个 + 数据位32个
uint16_t bufferID = 0;   // 数据存放的位置
volatile uint8_t rcvFlag = 0;       // 接收成功标志位
uint8_t num[4] = {0};    // 解析出的4字节数据

void InfraredRemote_Init(void)
{
    HAL_TIM_Base_Start(&htim3);               
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);   
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_3) return;

    if (isUpCapture)  // 如果是上升沿捕获
    {
        valueUp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        isUpCapture = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else  // 如果是下降沿捕获
    {
        valueDown = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        isUpCapture = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
        
        // 【核心优化】：巧妙利用16位无符号整型的自然溢出，直接省去繁琐的 upcount 计数和中断！
        // 因为脉宽最大值是同步码的 4.5ms（4500us），远小于 TIM3 的溢出周期 65535us。
        // 即使 valueDown < valueUp 发生了溢出减法，强转 uint16_t 也会得出正确的时间差。
        PluseWidth = (uint16_t)(valueDown - valueUp);

        // 同步码 (低电平9ms,高电平4.5ms。此处捕获的高电平宽度)
        if (PluseWidth > 4000 && PluseWidth < 5000)
        {
            bufferID = 0;
            frameBuffer[bufferID++] = PluseWidth;
        }
        else if (bufferID > 0)   // 收到同步码后，开始收集数据
        {
            if (bufferID < 33)
            {
                frameBuffer[bufferID++] = PluseWidth;
            }
            if (bufferID >= 33)   // 1同步码头 + 32位数据
            {
                rcvFlag = 1;      // 设置接收完成标志
                bufferID = 0;     // 准备下一帧
                // 在 ISR 中释放信号量，唤醒 IR 处理任务 (CMSIS-RTOS v2 允许在 ISR 中调用)
                osSemaphoreRelease(Sem_IR_Ready);
            }
        }
    }
}

// 将 buffer 中的数据转换为 4 字节数字存储在 num 中
static void bitbuffer_to_num(void)
{
    num[0] = num[1] = num[2] = num[3] = 0;
    for (int i = 0; i < 32; i++)
    {
        uint8_t byte_index = i / 8;
        num[byte_index] <<= 1;
        
        // 脉宽大于 1000us 即为逻辑 '1' (NEC数据1高电平1.68ms，数据0高电平0.56ms)
        if (frameBuffer[i + 1] >= 1000)
        {
            num[byte_index] |= 0x01;
        }
    }
}

uint16_t InfraredRemote_Get_KeyNum(void)
{
    if (rcvFlag == 1)
    {
        bitbuffer_to_num();
    }
    return (uint16_t)num[2]; // 取出键值对应的字节
}

/* 专门用于打印接收数据帧的函数。注意：这是一个阻塞性能杀手！ */
void InfraredRemote_DumpFrame(UART_HandleTypeDef *huart)
{
    if (huart == NULL || rcvFlag == 0) return;

    char printfbuff[160] = {0};
    
    // 【警告】：这里原先会触发30多次单字符 UART Transmit 和 sprintf，严重阻塞中断和主循环
    // 现已优化为将数据一次性拼接后批量发出，极大地减轻了堵塞时间。
    // 在生产环境/实际使用中，强烈建议在 main.c 中注释掉这个函数的调用！
    for (int i = 0; i < 33; i++)
    {
        char temp[8];
        sprintf(temp, "%u ", frameBuffer[i]);
        strcat(printfbuff, temp);
    }
    strcat(printfbuff, "\r\n");
    
    // 设置合理的超时，防止卡死
    HAL_UART_Transmit(huart, (uint8_t *)printfbuff, strlen(printfbuff), 20);
}
