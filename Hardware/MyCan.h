#ifndef __MYCAN_H
#define __MYCAN_H

#include "main.h"
#include "can.h"

#define CAN_ID_RIGHT_HAND   0x020
#define CAN_ID_LEFT_HAND    0x021
/* 函数声明 */

void MyCan_Init(void);
void MyCan_Start(void);
void MyCan_Stop(void);

/* 过滤器配置相关 */
HAL_StatusTypeDef MyCan_ConfigFilter_Mask32Bit(uint8_t filter_bank, uint32_t filter_id,
                                               uint32_t mask, uint8_t fifo);
HAL_StatusTypeDef MyCan_ConfigFilter_Mask16Bit(uint8_t filter_bank, uint16_t filter_id1,
                                               uint16_t filter_id2, uint32_t mask1,
                                               uint32_t mask2, uint8_t fifo);
HAL_StatusTypeDef MyCan_ConfigFilter_List32Bit(uint8_t filter_bank, uint32_t filter_id1,
                                               uint32_t filter_id2, uint8_t fifo);
HAL_StatusTypeDef MyCan_ConfigFilter_List16Bit(uint8_t filter_bank, uint16_t filter_id1,
                                               uint16_t filter_id2, uint16_t filter_id3,
                                               uint16_t filter_id4, uint8_t fifo);

/* 发送函数 */
HAL_StatusTypeDef MyCan_SendStdData(uint32_t std_id, uint8_t *data, uint8_t length);
HAL_StatusTypeDef MyCan_SendExtData(uint32_t ext_id, uint8_t *data, uint8_t length);

/* 统计信息 */
uint32_t MyCan_GetTxCount(void);
void MyCan_ResetCounters(void);

#endif /* __MYCAN_H */
