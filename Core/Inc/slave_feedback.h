#ifndef SLAVE_FEEDBACK_H
#define SLAVE_FEEDBACK_H

#include "main.h"
#include <stdbool.h>

#define SLAVE_FEEDBACK_ANGLE_COUNT 6U
#define SLAVE_FEEDBACK_FORCE_COUNT 5U
#define SLAVE_FEEDBACK_TIMEOUT_MS 150U

// SlaveFeedbackSnapshot_t：保存按拇指双关节、食指至小指排列的最近反馈
// 参数：有效位按字段排列；时间戳单位为毫秒，两块板及两种帧分别计时
// 返回值：无；无效字段仍可保留旧数值，使用者必须同时检查有效位
typedef struct {
    uint16_t angle_tenths[SLAVE_FEEDBACK_ANGLE_COUNT];
    uint8_t tangential_tenths[SLAVE_FEEDBACK_FORCE_COUNT];
    uint8_t normal_tenths[SLAVE_FEEDBACK_FORCE_COUNT];
    uint8_t angle_valid_mask;
    uint8_t force_valid_mask;
    uint8_t angle_sequence[2];
    uint8_t force_sequence[2];
    uint32_t angle_rx_ms[2];
    uint32_t force_rx_ms[2];
} SlaveFeedbackSnapshot_t;

// SlaveFeedbackStats_t：保存接收和启动异常统计，供调试器查看
// 参数：计数均为累计次数，不额外占用通信帧
// 返回值：无
typedef struct {
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    uint32_t queue_drops;
    uint32_t start_failures;
} SlaveFeedbackStats_t;

extern volatile SlaveFeedbackStats_t slaveFeedbackStats;

// SlaveFeedback_Init：创建反馈队列和任务，由任务启动及重试CAN2
// 参数：无
// 返回值：true表示软件资源创建成功，CAN启动失败不阻断原有动作控制
bool SlaveFeedback_Init(void);

// SlaveFeedback_OnCanRx：在CAN2中断回调中非阻塞投递帧及到达时间
// 参数：header - 接收头；data - 八字节存储区；rx_ms - 中断接收时刻
// 返回值：无；队列满时丢弃并计数
void SlaveFeedback_OnCanRx(const CAN_RxHeaderTypeDef *header,
                           const uint8_t data[8], uint32_t rx_ms);

// SlaveFeedback_GetSnapshot：复制缓存并按各帧到达时间清除过期有效位
// 参数：out - 输出快照；复制缓存后在临界区内取得当前HAL时刻
// 返回值：无；未收到或过期的数据对应有效位为0
void SlaveFeedback_GetSnapshot(SlaveFeedbackSnapshot_t *out);

#endif
