#include "MyCan.h"
#include <string.h>

/* 全局变量 */
static uint32_t g_tx_count = 0;

/**
  * @brief  CAN模块初始化 (适配FreeRTOS版本)
  * @retval None
  */
void MyCan_Init(void)
{
    g_tx_count = 0;
}

/**
  * @brief  启动CAN通信
  * @retval None
  */
void MyCan_Start(void)
{
    /* 启动CAN */
    HAL_CAN_Start(&hcan1);

    /* 激活接收中断（只激活FIFO0，FIFO1可选） */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  停止CAN通信
  * @retval None
  */
void MyCan_Stop(void)
{
    /* 停用所有中断 */
    HAL_CAN_DeactivateNotification(&hcan1,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_RX_FIFO1_MSG_PENDING
    );

    /* 停止CAN */
    HAL_CAN_Stop(&hcan1);
}

/* ========================= 过滤器配置函数 ========================= */

HAL_StatusTypeDef MyCan_ConfigFilter_Mask32Bit(uint8_t filter_bank, uint32_t filter_id,
                                               uint32_t mask, uint8_t fifo)
{
    CAN_FilterTypeDef filter_config = {0};

    filter_config.FilterBank = filter_bank;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = (filter_id << 5) >> 16;      // ID高16位
    filter_config.FilterIdLow = (filter_id << 5) & 0xFFFF;    // ID低16位
    filter_config.FilterMaskIdHigh = (mask << 5) >> 16;       // 掩码高16位
    filter_config.FilterMaskIdLow = (mask << 5) & 0xFFFF;     // 掩码低16位
    filter_config.FilterFIFOAssignment = fifo;
    filter_config.FilterActivation = ENABLE;
    filter_config.SlaveStartFilterBank = 14;  // CAN1使用0-13，CAN2使用14-27

    return HAL_CAN_ConfigFilter(&hcan1, &filter_config);
}

HAL_StatusTypeDef MyCan_ConfigFilter_Mask16Bit(uint8_t filter_bank, uint16_t filter_id1,
                                               uint16_t filter_id2, uint32_t mask1,
                                               uint32_t mask2, uint8_t fifo)
{
    CAN_FilterTypeDef filter_config = {0};

    filter_config.FilterBank = filter_bank;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_16BIT;
    filter_config.FilterIdHigh = filter_id1 << 5;
    filter_config.FilterIdLow = filter_id2 << 5;
    filter_config.FilterMaskIdHigh = mask1 << 5;
    filter_config.FilterMaskIdLow = mask2 << 5;
    filter_config.FilterFIFOAssignment = fifo;
    filter_config.FilterActivation = ENABLE;
    filter_config.SlaveStartFilterBank = 14;

    return HAL_CAN_ConfigFilter(&hcan1, &filter_config);
}

HAL_StatusTypeDef MyCan_ConfigFilter_List32Bit(uint8_t filter_bank, uint32_t filter_id1,
                                               uint32_t filter_id2, uint8_t fifo)
{
    CAN_FilterTypeDef filter_config = {0};

    filter_config.FilterBank = filter_bank;
    filter_config.FilterMode = CAN_FILTERMODE_IDLIST;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = (filter_id1 << 5) >> 16;
    filter_config.FilterIdLow = (filter_id1 << 5) & 0xFFFF;
    filter_config.FilterMaskIdHigh = (filter_id2 << 5) >> 16;
    filter_config.FilterMaskIdLow = (filter_id2 << 5) & 0xFFFF;
    filter_config.FilterFIFOAssignment = fifo;
    filter_config.FilterActivation = ENABLE;
    filter_config.SlaveStartFilterBank = 14;

    return HAL_CAN_ConfigFilter(&hcan1, &filter_config);
}

HAL_StatusTypeDef MyCan_ConfigFilter_List16Bit(uint8_t  filter_bank,
                                          uint16_t id1,
                                          uint16_t id2,
                                          uint16_t id3,
                                          uint16_t id4,
                                          uint8_t  fifo)
{
    CAN_FilterTypeDef cf = {0};

    if ((id1 > 0x7FF) || (id2 > 0x7FF) || (id3 > 0x7FF) || (id4 > 0x7FF))
        return HAL_ERROR;

    #define SHIFT_ID(x) ((uint16_t)((x) << 5))   // b15~b5 = STDID[10:0]，b4~b0 = 0

    cf.FilterBank             = filter_bank;
    cf.FilterMode             = CAN_FILTERMODE_IDLIST;   // 列表模式
    cf.FilterScale            = CAN_FILTERSCALE_16BIT;   // 16 位尺度
    cf.FilterFIFOAssignment   = fifo;
    cf.FilterActivation       = ENABLE;

    cf.FilterIdHigh      = SHIFT_ID(id1);
    cf.FilterIdLow       = SHIFT_ID(id2);
    cf.FilterMaskIdHigh  = SHIFT_ID(id3);
    cf.FilterMaskIdLow   = SHIFT_ID(id4);

    cf.SlaveStartFilterBank = 14; 

    return HAL_CAN_ConfigFilter(&hcan1, &cf);
}

/* ========================= 发送函数 ========================= */

HAL_StatusTypeDef MyCan_SendStdData(uint32_t std_id, uint8_t *data, uint8_t length)
{
    if (length > 8) return HAL_ERROR;

    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);

    if (status == HAL_OK) {
        g_tx_count++;
    }

    return status;
}

HAL_StatusTypeDef MyCan_SendExtData(uint32_t ext_id, uint8_t *data, uint8_t length)
{
    if (length > 8) return HAL_ERROR;

    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;

    tx_header.ExtId = ext_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);

    if (status == HAL_OK) {
        g_tx_count++;
    }

    return status;
}

/* ========================= 统计函数 ========================= */

uint32_t MyCan_GetTxCount(void)
{
    return g_tx_count;
}

void MyCan_ResetCounters(void)
{
    g_tx_count = 0;
}
