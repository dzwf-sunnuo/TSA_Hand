#include "cmsis_os2.h"
#include "main.h"
#include "rs485.h"
#include "motion.h"
#include "FreeRTOS.h"
#include "app_comm_tasks.h"   // Sem_NewMotionCmd

  uint16_t Action1_ban1[13][5]={
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                       {0x1332, 0x1A32, 0x0820, 0x0000, 0x0101},
                                       {0x1532, 0x1A32, 0x1020, 0x0000, 0x0101},
                                       {0x1932, 0x1A32, 0x1520, 0x0000, 0x0101},
                                       {0x1932, 0x1A32, 0x1520, 0x0000, 0x0101},
                                       {0x1932, 0x1A32, 0x1520, 0x0000, 0x0101},
                                       {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                       {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                       {0x1632, 0x1E32, 0x1532, 0x0000, 0x0101},
                                       {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                       {0x1932, 0x0832, 0x0B20, 0x0000, 0x0101},
                                       {0x1932, 0x0832, 0x0B64, 0x0000, 0x0101},
                                       {0x0064, 0x0064, 0x0064, 0x0000, 0x0101}};


 uint16_t Action1_ban2[13][5]={{0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
                                       {0x2964, 0x2564, 0x2564, 0x2664, 0x0101},
                                       {0x0064, 0x2564, 0x2564, 0x2664, 0x0101},
                                       {0x0064, 0x0064, 0x2564, 0x2664, 0x0101},
                                       {0x0064, 0x0064, 0x0064, 0x2664, 0x0101},
                                       {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
                                       {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
                                       {0x2964, 0x2564, 0x2564, 0x0064, 0x0101},
                                       {0x1b64, 0x1864, 0x2064, 0x2664, 0x0101},
                                       {0x0064, 0x2564, 0x2564, 0x2664, 0x0101},
                                       {0x1C64, 0x2564, 0x2564, 0x2664, 0x0101},
                                       {0x2964, 0x2564, 0x2564, 0x2664, 0x0101},
                                       {0x0064, 0x0064, 0x0064, 0x0064, 0x0101}};


 uint16_t Action2_ban1[9][5]={
                                         {0x0050, 0x0050, 0x0032, 0x0000, 0x0101},
                                         {0x0050, 0x0050, 0x0032, 0x0000, 0x0101},
                                         {0x0050, 0x0050, 0x0032, 0x0000, 0x0101},
                                         {0x0050, 0x0050, 0x0032, 0x0000, 0x0101},
                                         {0x0050, 0x0050, 0x0032, 0x0000, 0x0101},
                                         {0x1950, 0x2550, 0x1050, 0x0000, 0x0101},
                                         {0x1950, 0x2550, 0x1050, 0x0000, 0x0101},
                                         {0x1950, 0x2550, 0x1050, 0x0000, 0x0101},
                                         {0x0050, 0x0050, 0x0032, 0x0000, 0x0101},
 };


 uint16_t Action2_ban2[9][5]={
                                         {0x0050, 0x0050, 0x0050, 0x0050, 0x0101},
                                         {0x2950, 0x0050, 0x0050, 0x0050, 0x0101},
                                         {0x2950, 0x2550, 0x0050, 0x0050, 0x0101},
                                         {0x2950, 0x2550, 0x2550, 0x0050, 0x0101},
                                         {0x0050, 0x2550, 0x2550, 0x2750, 0x0101},
                                         {0x0050, 0x0050, 0x2550, 0x2750, 0x0101},
                                         {0x0050, 0x0050, 0x0050, 0x2750, 0x0101},
                                         {0x0050, 0x0050, 0x0050, 0x0050, 0x0101},
                                         {0x0050, 0x0050, 0x0050, 0x0050, 0x0101},
};

 uint16_t Action3_ban1[6][5]={
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                         {0x1764, 0x1B64, 0x1264, 0x0000, 0x0101},
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
 };


 uint16_t Action3_ban2[6][5]={
                                         {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
                                         {0x2964, 0x2564, 0x2564, 0x2764, 0x0101},
                                         {0x1D64, 0x0064, 0x0064, 0x0064, 0x0101},
                                         {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
                                         {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
                                         {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
};

 uint16_t Action4_ban1[3][5]={
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                         {0x1932, 0x0832, 0x0B64, 0x0000, 0x0101},
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
 };


 uint16_t Action4_ban2[3][5]={
                                         {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
                                         {0x2964, 0x2564, 0x2564, 0x2664, 0x0101},
                                         {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
};

uint16_t shake_hand1[5] = {0x1064,0x0C64,0x0864,0x0000,0x0101};
uint16_t shake_hand2[5] = {0x0C64,0x0E64,0x0964,0x1064,0x0101};

void finger_motion(uint8_t num, uint16_t speed)
{
    // 排空旧信号，确保只响应动画开始后的新指令
    while (osSemaphoreAcquire(Sem_NewMotionCmd, 0) == osOK);

    uint8_t i = 0;
    if (num == COUNT_NUMBERS) {
        for (i = 0; i < 13; i++) {
            Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, Action1_ban1[i]);
            Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, Action1_ban2[i]);
            HAL_Delay(speed);
            if (osSemaphoreAcquire(Sem_NewMotionCmd, 0) == osOK) break;
        }
    } else if (num == CONTINUOUS_BENDING) {
        for (i = 0; i < 9; i++) {
            Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, Action2_ban1[i]);
            Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, Action2_ban2[i]);
            HAL_Delay(speed);
            if (osSemaphoreAcquire(Sem_NewMotionCmd, 0) == osOK) break;
        }
    } else if (num == OTHER_MOTION) {
        for (i = 0; i < 6; i++) {
            Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, Action3_ban1[i]);
            Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, Action3_ban2[i]);
            HAL_Delay(speed);
            if (osSemaphoreAcquire(Sem_NewMotionCmd, 0) == osOK) break;
        }
    } else if (num == CONTINUOUS_FISTING) {
        for (i = 0; i < 3; i++) {
            Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, Action4_ban1[i]);
            Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, Action4_ban2[i]);
            HAL_Delay(speed);
            if (osSemaphoreAcquire(Sem_NewMotionCmd, 0) == osOK) break;
        }
    }
}
/*//0513 右手
 uint16_t Single_Action_ban1[4][5]={
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                         {0x1364, 0x0D64, 0x0964,0x0000, 0x0101},// 握手
                                         {0x1564, 0x0D64, 0x0864, 0x0000, 0x0101},//ok
										 {0x1564, 0x0F64, 0x0D64, 0x0000, 0x0101},//兰花指

 };


 uint16_t Single_Action_ban2[4][5]={
                                         {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
                                         {0x0D64,0x0F64,0x0A64,0x1164,0x0101},// 握手
                                         {0x1964, 0x0060, 0x0064, 0x0064, 0x0101},//ok
										 {0x0064, 0x1C60, 0x0064, 0x0064, 0x0101},//兰花指

};*/
//0515 左手：握手0E64 0C64 0864 0000 0101    0964 0A60 0A64 0B64 0101
//ok  1564 0C64 0864 0000 0101    1764 0060 0064 0064 0101
//兰花指  1564 1064 0D64 0000 0101    0064 1B60 0064 0064 0101
 uint16_t Single_Action_ban1[4][5]={
                                         {0x0064, 0x0064, 0x0064, 0x0000, 0x0101},
                                         {0x0E64, 0x0C64, 0x0864, 0x0000, 0x0101},// 握手
                                         {0x1564, 0x1264, 0x1064, 0x0000, 0x0101},//ok
										 {0x1764, 0x1464, 0x1364, 0x0000, 0x0101},//兰花指

 };


 uint16_t Single_Action_ban2[4][5]={
                                         {0x0064, 0x0064, 0x0064, 0x0064, 0x0101},
                                         {0x0964, 0x0A60, 0x0A64, 0x0B64, 0x0101},// 握手
                                         {0x1864, 0x0060, 0x0064, 0x0064, 0x0101},//ok
										 {0x0064, 0x1B60, 0x0064, 0x0064, 0x0101},//兰花指

};

/* 单个动作用查表: [编号] = { ban1_数据指针, ban2_数据指针 } */
static const struct {
    uint16_t *ban1;
    uint16_t *ban2;
} single_action_table[] = {
    [0x00] = { Single_Action_ban1[0], Single_Action_ban2[0] },  // 展开
    [0x01] = { Single_Action_ban1[1], Single_Action_ban2[1]},  // 握手
    [0x02] = { Single_Action_ban1[2], Single_Action_ban2[2] },  // 0k
    [0x03] = { Single_Action_ban1[3], Single_Action_ban2[3] },  // 兰花指
    [0x04] = { Single_Action_ban1[4], Single_Action_ban2[4] },  // 4
    [0x05] = { Action1_ban1[6], Action1_ban2[6]  },  // 5
    [0x06] = { Action1_ban1[7], Action1_ban2[7]  },  // 6
    [0x07] = { Action1_ban1[8], Action1_ban2[8]  },  // 7
    [0x08] = { Action1_ban1[9], Action1_ban2[9]  },  // 8
    [0x09] = { Action1_ban1[10],Action1_ban2[1]  },  // 9
    [0x0A] = { Action3_ban1[1], Action3_ban2[1]  },  // 拳
    [0x0B] = { Action3_ban1[2], Action3_ban2[2]  },  // OK
};
#define SINGLE_ACTION_COUNT (sizeof(single_action_table) / sizeof(single_action_table[0]))

void finger_single_action(uint16_t num)
{
    if (num >= SINGLE_ACTION_COUNT) return;

    Host_write16_slave(0x01, 0x10, 0x0000, 0x0005, 0x0A, single_action_table[num].ban1);
    osDelay(15);
    Host_write16_slave(0x02, 0x10, 0x0000, 0x0005, 0x0A, single_action_table[num].ban2);
}
