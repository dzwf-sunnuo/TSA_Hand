#ifndef __MYCAN_H
#define __MYCAN_H

#include "main.h"
#include "can.h"

/* CAN消息结构体 */
typedef struct {
    uint32_t id;                // CAN ID
    uint8_t is_extended;       // 是否为扩展帧: 0-标准帧, 1-扩展帧
    uint8_t data[8];           // 数据
    uint8_t length;            // 数据长度 (0-8)
    uint32_t timestamp;        // 时间戳（收到消息时的tick）
} CAN_Message_t;

/* 接收回调函数类型 */
typedef void (*CAN_RxCallback_t)(CAN_Message_t *msg);

/* 函数声明 */
void MyCan_Init(CAN_HandleTypeDef *hcan, CAN_RxCallback_t callback);
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
HAL_StatusTypeDef MyCan_SendStdRemote(uint32_t std_id, uint8_t length);
HAL_StatusTypeDef MyCan_SendExtRemote(uint32_t ext_id, uint8_t length);

/* 中断回调函数（需要在stm32f4xx_it.c中调用） */
void MyCan_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void MyCan_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

/* 统计信息 */
uint32_t MyCan_GetRxCount(void);
uint32_t MyCan_GetTxCount(void);
void MyCan_ResetCounters(void);

#endif /* __MYCAN_H */

///* /* 示例1：配置过滤器组0 - 接收ID 0x100的标准帧 */
//    MyCan_ConfigFilter_Mask32Bit(0, 0x100, 0x7FF, CAN_RX_FIFO0);
//    // 掩码0x7FF表示精确匹配，只接收ID为0x100的帧
//    
//    /* 示例2：配置过滤器组1 - 接收ID范围0x200-0x20F的标准帧 */
//    MyCan_ConfigFilter_Mask32Bit(1, 0x200, 0x7F0, CAN_RX_FIFO0);
//    // 掩码0x7F0表示ID的低4位不关心，接收0x200-0x20F
//    
//    /* 示例3：配置过滤器组2 - 接收ID 0x300和0x301的标准帧（列表模式） */
//    MyCan_ConfigFilter_List32Bit(2, 0x300, 0x301, CAN_RX_FIFO0);
//    
//    /* 示例4：配置过滤器组3 - 接收所有标准帧（掩码全0） */
//    MyCan_ConfigFilter_Mask32Bit(3, 0x000, 0x000, CAN_RX_FIFO0);
    
