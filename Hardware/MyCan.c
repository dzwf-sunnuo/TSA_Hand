#include "MyCan.h"
#include <string.h>

/* 全局变量 */
static CAN_HandleTypeDef *g_hcan = NULL;
static CAN_RxCallback_t g_rx_callback = NULL;
static uint32_t g_rx_count = 0;
static uint32_t g_tx_count = 0;

/**
  * @brief  CAN模块初始化
  * @param  hcan: CAN句柄指针
  * @param  callback: 接收回调函数指针
  * @retval None
  */
void MyCan_Init(CAN_HandleTypeDef *hcan, CAN_RxCallback_t callback)
{
    g_hcan = hcan;
    g_rx_callback = callback;
    g_rx_count = 0;
    g_tx_count = 0;
}

/**
  * @brief  启动CAN通信
  * @retval None
  */
void MyCan_Start(void)
{
    if (g_hcan == NULL) return;
    
    /* 启动CAN */
    HAL_CAN_Start(g_hcan);
    
    /* 激活接收中断（只激活FIFO0，FIFO1可选） */
    HAL_CAN_ActivateNotification(g_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    // 如果需要FIFO1也激活接收中断，取消下面注释：
    // HAL_CAN_ActivateNotification(g_hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
  * @brief  停止CAN通信
  * @retval None
  */
void MyCan_Stop(void)
{
    if (g_hcan == NULL) return;
    
    /* 停用所有中断 */
    HAL_CAN_DeactivateNotification(g_hcan, 
        CAN_IT_RX_FIFO0_MSG_PENDING | 
        CAN_IT_RX_FIFO1_MSG_PENDING
    );
    
    /* 停止CAN */
    HAL_CAN_Stop(g_hcan);
}

/* ========================= 过滤器配置函数 ========================= */

/**
  * @brief  配置32位掩码模式过滤器
  * @param  filter_bank: 过滤器组编号 (0-13 for CAN1, 14-27 for CAN2)
  * @param  filter_id: 过滤器ID
  * @param  mask: 掩码
  * @param  fifo: FIFO分配 (CAN_RX_FIFO0 或 CAN_RX_FIFO1)
  * @retval HAL状态
  * 
  * @note 掩码模式说明:
  * - 掩码位为0: 对应ID位不关心(可以是0或1)
  * - 掩码位为1: 对应ID位必须与过滤器ID匹配
  * 例如:
  * filter_id = 0x123, mask = 0x7F0
  * 表示接收ID为0x120-0x12F范围内的帧
  */
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
    
    return HAL_CAN_ConfigFilter(g_hcan, &filter_config);
}

/**
  * @brief  配置16位掩码模式过滤器（双ID）
  * @param  filter_bank: 过滤器组编号
  * @param  filter_id1: 第一个ID
  * @param  filter_id2: 第二个ID
  * @param  mask1: 第一个掩码
  * @param  mask2: 第二个掩码
  * @param  fifo: FIFO分配
  * @retval HAL状态
  */
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
    
    return HAL_CAN_ConfigFilter(g_hcan, &filter_config);
}

/**
  * @brief  配置32位列表模式过滤器（精确匹配两个ID）
  * @param  filter_bank: 过滤器组编号
  * @param  filter_id1: 第一个ID
  * @param  filter_id2: 第二个ID
  * @param  fifo: FIFO分配
  * @retval HAL状态
  */
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
    
    return HAL_CAN_ConfigFilter(g_hcan, &filter_config);
}

/**
 * @brief  配置16位列表模式过滤器（精确匹配 4 个标准帧ID）
 * @param  filter_bank : 0 ~ 13（F103 只有 14 组，0~13）
 * @param  id1 ~ id4   : 11 位标准 ID（0x000~0x7FF）
 * @param  fifo        : CAN_FilterFIFO0 或 CAN_FilterFIFO1
 * @retval HAL 状态
 */
HAL_StatusTypeDef MyCan_ConfigFilter_List16Bit(uint8_t  filter_bank,
                                          uint16_t id1,
                                          uint16_t id2,
                                          uint16_t id3,
                                          uint16_t id4,
                                          uint8_t  fifo)
{
    CAN_FilterTypeDef cf = {0};

    /* 1. 检查 ID 范围 */
    if ((id1 > 0x7FF) || (id2 > 0x7FF) || (id3 > 0x7FF) || (id4 > 0x7FF))
        return HAL_ERROR;

    /* 2. 把 11 位标准 ID 放到 16 位槽的高 11 位，并清 RTR/IDE 位 */
    #define SHIFT_ID(x) ((uint16_t)((x) << 5))   // b15~b5 = STDID[10:0]，b4~b0 = 0

    cf.FilterBank             = filter_bank;
    cf.FilterMode             = CAN_FILTERMODE_IDLIST;   // 列表模式
    cf.FilterScale            = CAN_FILTERSCALE_16BIT;   // 16 位尺度
    cf.FilterFIFOAssignment   = fifo;
    cf.FilterActivation       = ENABLE;

    /*
     * 16 位列表模式下，4 个寄存器被当成 4 个独立的“ID 槽”：
     *   FilterIdHigh  → 槽 0
     *   FilterIdLow   → 槽 1
     *   FilterMaskIdHigh → 槽 2
     *   FilterMaskIdLow  → 槽 3
     * 每个槽 16 位：b15~b5 放 STDID[10:0]，b4(RTR) b3(IDE) 通常清 0。
     */
    cf.FilterIdHigh      = SHIFT_ID(id1);
    cf.FilterIdLow       = SHIFT_ID(id2);
    cf.FilterMaskIdHigh  = SHIFT_ID(id3);
    cf.FilterMaskIdLow   = SHIFT_ID(id4);

    cf.SlaveStartFilterBank = 14;   // 单 CAN 环境固定 14 即可

    return HAL_CAN_ConfigFilter(&hcan1, &cf);
}
/* ========================= 发送函数 ========================= */

/**
  * @brief  发送标准数据帧
  * @param  std_id: 标准ID (11位)
  * @param  data: 数据指针
  * @param  length: 数据长度 (0-8)
  * @retval HAL状态
  */
HAL_StatusTypeDef MyCan_SendStdData(uint32_t std_id, uint8_t *data, uint8_t length)
{
    if (g_hcan == NULL || length > 8) return HAL_ERROR;
    
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;
    
    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;
    
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(g_hcan, &tx_header, data, &tx_mailbox);
    
    if (status == HAL_OK) {
        g_tx_count++;
    }
    
    return status;
}

/**
  * @brief  发送扩展数据帧
  * @param  ext_id: 扩展ID (29位)
  * @param  data: 数据指针
  * @param  length: 数据长度
  * @retval HAL状态
  */
HAL_StatusTypeDef MyCan_SendExtData(uint32_t ext_id, uint8_t *data, uint8_t length)
{
    if (g_hcan == NULL || length > 8) return HAL_ERROR;
    
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;
    
    tx_header.ExtId = ext_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;
    
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(g_hcan, &tx_header, data, &tx_mailbox);
    
    if (status == HAL_OK) {
        g_tx_count++;
    }
    
    return status;
}

/**
  * @brief  发送标准远程帧
  * @param  std_id: 标准ID
  * @param  length: 请求的数据长度
  * @retval HAL状态
  */
HAL_StatusTypeDef MyCan_SendStdRemote(uint32_t std_id, uint8_t length)
{
    if (g_hcan == NULL || length > 8) return HAL_ERROR;
    
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;
    
    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_REMOTE;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;
    
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(g_hcan, &tx_header, NULL, &tx_mailbox);
    
    if (status == HAL_OK) {
        g_tx_count++;
    }
    
    return status;
}

/**
  * @brief  发送扩展远程帧
  * @param  ext_id: 扩展ID
  * @param  length: 请求的数据长度
  * @retval HAL状态
  */
HAL_StatusTypeDef MyCan_SendExtRemote(uint32_t ext_id, uint8_t length)
{
    if (g_hcan == NULL || length > 8) return HAL_ERROR;
    
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;
    
    tx_header.ExtId = ext_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_REMOTE;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;
    
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(g_hcan, &tx_header, NULL, &tx_mailbox);
    
    if (status == HAL_OK) {
        g_tx_count++;
    }
    
    return status;
}

/* ========================= 中断回调函数 ========================= */

/**
  * @brief  FIFO0接收中断回调函数
  * @param  hcan: CAN句柄指针
  * @retval None
  */
void MyCan_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    CAN_Message_t message;
    
    /* 读取CAN消息 */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        /* 填充消息结构 */
        if (rx_header.IDE == CAN_ID_STD) {
            message.id = rx_header.StdId;
            message.is_extended = 0;
        } else {
            message.id = rx_header.ExtId;
            message.is_extended = 1;
        }
        
        message.length = rx_header.DLC;
        memcpy(message.data, rx_data, message.length);
        message.timestamp = HAL_GetTick();
        
        /* 更新接收计数器 */
        g_rx_count++;
        
        /* 调用用户回调函数 */
        if (g_rx_callback != NULL) {
            g_rx_callback(&message);
        }
    }
}

/**
  * @brief  FIFO1接收中断回调函数
  * @param  hcan: CAN句柄指针
  * @retval None
  */
void MyCan_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    CAN_Message_t message;
    
    /* 读取CAN消息 */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK) {
        /* 填充消息结构 */
        if (rx_header.IDE == CAN_ID_STD) {
            message.id = rx_header.StdId;
            message.is_extended = 0;
        } else {
            message.id = rx_header.ExtId;
            message.is_extended = 1;
        }
        
        message.length = rx_header.DLC;
        memcpy(message.data, rx_data, message.length);
        message.timestamp = HAL_GetTick();
        
        /* 更新接收计数器 */
        g_rx_count++;
        
        /* 调用用户回调函数 */
        if (g_rx_callback != NULL) {
            g_rx_callback(&message);
        }
    }
}

/* ========================= 统计函数 ========================= */

/**
  * @brief  获取接收消息计数
  * @retval 接收消息数量
  */
uint32_t MyCan_GetRxCount(void)
{
    return g_rx_count;
}

/**
  * @brief  获取发送消息计数
  * @retval 发送消息数量
  */
uint32_t MyCan_GetTxCount(void)
{
    return g_tx_count;
}

/**
  * @brief  重置计数器
  * @retval None
  */
void MyCan_ResetCounters(void)
{
    g_rx_count = 0;
    g_tx_count = 0;
}


