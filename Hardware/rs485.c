#include "rs485.h"
#include "usart.h"
#include "stdio.h"
#include "wrist.h"
#include "app_comm_tasks.h"
#include <string.h>

extern volatile float motion_speed_factor;    // 运动速度倍率因子
extern volatile uint16_t cycle_time;          // 摆动手腕的循环时间

MODBUS modbus1; // 用于控制手指驱动板的 MODBUS 结构体 (主机模式)
MODBUS modbus2; // 用于接收 PC 上位机的 MODBUS 结构体 (从机模式)

uint16_t Reg[100] = {0}; // 模拟 Modbus 共享寄存器，16位数据存储区

volatile uint8_t pending_slave_dispatch = 1; // 标志：是否有待处理的从机下发任务

/**
 * @brief 发送单个字节数据到 USART2 (PC端)
 */
void modbus2_Send_Byte(uint8_t ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xff);
}

/**
 * @brief 发送单个字节数据到 USART1 (手指端)
 */
void modbus1_Send_Byte(uint8_t ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xff);
}

/**
 * @brief Modbus2 (接收PC) 初始化
 */
void modbus2_Init()
{
    memset(&modbus2, 0, sizeof(MODBUS));
    
    Reg[0] = 0x0000;
    Reg[1] = 0x0000;
    Reg[2] = 0x0000;
    Reg[3] = 0x0000;
    Reg[4] = 0x0000;
    Reg[5] = 0x0000;
    Reg[6] = 0x0000;
    Reg[7] = 0x0000;
    Reg[8] = 0x0000;
    Reg[9] = 0x0000;
    Reg[10] = 0x0000;
    Reg[11] = 0x0000;
    Reg[12] = 0x0000;
    Reg[13] = 0x0000;
    Reg[14] = 0x0001;
    Reg[15] = 0x0898; // 默认2200ms
    Reg[16] = 0x012C;
    Reg[17] = 0x06A4;

    modbus2.myadd = 0x10; // PC 端发往本板的地址为 0x10
    modbus2.timrun = 0;
}

/**
 * @brief Modbus2 事件处理函数 (响应PC端请求)
 * @note  只有本函数在解析到有效的上位机写入指令时，才会触发向手指板的分发。
 */
void modbus2_Event()
{
    uint16_t crc, rccrc;
    if(modbus2.reflag == 0){return;} // 检查一帧数据是否接收完成
    
    // 计算接收缓冲区中除最后两位 CRC 以外的数据校验和
    crc = Modbus_CRC16(&modbus2.rcbuf[0], modbus2.recount - 2);
    // 从接收缓冲区提取发来的 CRC
    rccrc = modbus2.rcbuf[modbus2.recount - 2] * 256 + modbus2.rcbuf[modbus2.recount - 1];
    
    if(crc == rccrc)
    {
        // 如果地址匹配或为广播地址 0x00
        if(modbus2.rcbuf[0] == modbus2.myadd || modbus2.rcbuf[0] == 0x00)
        {
            switch(modbus2.rcbuf[1])
            {
                case 3:  modbus2_Func3();  break; // 读寄存器
                case 6:  modbus2_Func6();  break; // 写单寄存器
                case 16: modbus2_Func16(); break; // 写多寄存器
            }
            
            /* 
             * 【重点】：触发向下游手指驱动板的数据下发任务！
             * 在 RTOS 架构下，我们不再使用状态机延时，只给上层 Task 传递标志。
             */
            pending_slave_dispatch = 1; 
        }
    }

    // 释放接收窗口，准备接收下一帧
    modbus2.recount = 0;
    modbus2.reflag = 0;

    /* 本地手腕控制逻辑处理 */
    if(Reg[14] % 256 == 0x01){
        vel_move(Reg[0]/256, Reg[0]%256, Reg[1]/256, Reg[1]%256);
        vel_move1(Reg[2]/256, Reg[2]%256, Reg[3]/256, Reg[3]%256);
    }

    /* 队列驱动：直接发消息，WristMotion_Start/Stop 本身是幂等的 */
    uint8_t new_enable = Reg[14] / 256;
    if (new_enable) {
        WristMotion_Start(MODE_PENDULUM);
    } else {
        WristMotion_Stop();
    }
    cycle_time = Reg[15];
}

/**
 * @brief PC端读取寄存器 (功能码 0x03)
 */
void modbus2_Func3()
{
    uint16_t Regadd, Reglen, crc;
    uint16_t i, j;
    Regadd = modbus2.rcbuf[2]*256 + modbus2.rcbuf[3];
    Reglen = modbus2.rcbuf[4]*256 + modbus2.rcbuf[5];
    i = 0;
    modbus2.sendbuf[i++] = modbus2.myadd;
    modbus2.sendbuf[i++] = 0x03;
    modbus2.sendbuf[i++] = ((Reglen*2)%256);
    
    // 返回对应的寄存器值
    for(j=0; j<Reglen; j++)
    {
        modbus2.sendbuf[i++] = Reg[Regadd+j]/256;
        modbus2.sendbuf[i++] = Reg[Regadd+j]%256;
    }
    
    crc = Modbus_CRC16(modbus2.sendbuf, i);
    modbus2.sendbuf[i++] = crc/256;
    modbus2.sendbuf[i++] = crc%256;
    
    for(j=0; j<i; j++){modbus2_Send_Byte(modbus2.sendbuf[j]);}
}

/**
 * @brief PC端写单寄存器 (功能码 0x06)
 */
void modbus2_Func6()
{
    uint16_t Regadd, val, crc;
    uint16_t i, j;
    Regadd = modbus2.rcbuf[2]*256 + modbus2.rcbuf[3];
    val = modbus2.rcbuf[4]*256 + modbus2.rcbuf[5];
    Reg[Regadd] = val;
    i = 0;
    modbus2.sendbuf[i++] = modbus2.myadd;
    modbus2.sendbuf[i++] = 0x06;
    modbus2.sendbuf[i++] = Regadd/256;
    modbus2.sendbuf[i++] = Regadd%256;
    modbus2.sendbuf[i++] = val/256;
    modbus2.sendbuf[i++] = val%256;
    crc = Modbus_CRC16(modbus2.sendbuf, i);
    modbus2.sendbuf[i++] = crc/256;
    modbus2.sendbuf[i++] = crc%256;
    
    for(j=0; j<i; j++){modbus2_Send_Byte(modbus2.sendbuf[j]);}
}

/**
 * @brief PC端写多寄存器 (功能码 0x10)
 */
void modbus2_Func16()
{
    uint16_t Regadd, Reglen, crc;
    uint16_t i, j;
    Regadd = modbus2.rcbuf[2]*256 + modbus2.rcbuf[3];
    Reglen = modbus2.rcbuf[4]*256 + modbus2.rcbuf[5];
    
    // 更新本地寄存器
    for(i=0; i<Reglen; i++)
    {
        Reg[Regadd+i] = modbus2.rcbuf[7+i*2]*256 + modbus2.rcbuf[8+i*2];
    }
    
    modbus2.sendbuf[0] = modbus2.rcbuf[0];
    modbus2.sendbuf[1] = modbus2.rcbuf[1];
    modbus2.sendbuf[2] = modbus2.rcbuf[2];
    modbus2.sendbuf[3] = modbus2.rcbuf[3];
    modbus2.sendbuf[4] = modbus2.rcbuf[4];
    modbus2.sendbuf[5] = modbus2.rcbuf[5];
    crc = Modbus_CRC16(modbus2.sendbuf, 6);
    modbus2.sendbuf[6] = crc/256;
    modbus2.sendbuf[7] = crc%256;
    
    for(j=0; j<8; j++){modbus2_Send_Byte(modbus2.sendbuf[j]);}
}

/**
 * @brief 作为主机控制从机写单寄存器 (功能码 0x06)
 */
void Host_write06_slave(uint8_t slave, uint8_t fun, uint16_t StartAddr, uint16_t num)
{
    uint16_t crc;
    modbus1.slave_add = slave;
    modbus1.Host_Txbuf[0] = slave;
    modbus1.Host_Txbuf[1] = fun;
    modbus1.Host_Txbuf[2] = StartAddr/256;
    modbus1.Host_Txbuf[3] = StartAddr%256;
    modbus1.Host_Txbuf[4] = num/256;
    modbus1.Host_Txbuf[5] = num%256;
    crc = Modbus_CRC16(&modbus1.Host_Txbuf[0], 6);
    modbus1.Host_Txbuf[6] = crc/256;
    modbus1.Host_Txbuf[7] = crc%256;
    
    HAL_UART_Transmit(&huart1, modbus1.Host_Txbuf, 8, 100);
    modbus1.Host_send_flag = 1;
}

/**
 * @brief 作为主机控制从机写多寄存器 (功能码 0x10)
 * @note  已优化为 DMA/批量发送，极大减少等待时间
 */
void Host_write16_slave(uint8_t slave, uint8_t fun, uint16_t StartAddr, uint16_t REnum, uint8_t BYTEnum, uint16_t* ModbusData)
{
    uint16_t crc;
    uint16_t send_len = 0;
    
    modbus1.slave_add = slave;
    modbus1.Host_Txbuf[send_len++] = slave;
    modbus1.Host_Txbuf[send_len++] = fun;
    modbus1.Host_Txbuf[send_len++] = StartAddr/256;
    modbus1.Host_Txbuf[send_len++] = StartAddr%256;
    modbus1.Host_Txbuf[send_len++] = REnum/256;
    modbus1.Host_Txbuf[send_len++] = REnum%256;
    modbus1.Host_Txbuf[send_len++] = BYTEnum;

    // 动态拼接传入的手指从机数据
    for (int k = 0; k < REnum; k++) {
        modbus1.Host_Txbuf[send_len++] = ModbusData[k]/256;
        modbus1.Host_Txbuf[send_len++] = ModbusData[k]%256;
    }

    // 计算整帧 CRC
    crc = Modbus_CRC16(&modbus1.Host_Txbuf[0], send_len);
    modbus1.Host_Txbuf[send_len++] = crc/256;
    modbus1.Host_Txbuf[send_len++] = crc%256;

    // 批量发送至 USART1 手指驱动总线，超时100ms
    HAL_UART_Transmit(&huart1, modbus1.Host_Txbuf, send_len, 100);
    modbus1.Host_send_flag = 1;
}
