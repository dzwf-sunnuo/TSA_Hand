#include "slave_feedback.h"
#include "can.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define FEEDBACK_BOARD1_ANGLE_ID 0x020U
#define FEEDBACK_BOARD1_FORCE_ID 0x021U
#define FEEDBACK_BOARD2_ANGLE_ID 0x030U
#define FEEDBACK_BOARD2_FORCE_ID 0x031U

// SlaveFeedbackRx_t：在队列中保留原始帧和中断到达时间
// 参数：header保留IDE、RTR和DLC，防止把其他帧误当作反馈
// 返回值：无
typedef struct {
    CAN_RxHeaderTypeDef header;
    uint32_t rx_ms;
    uint8_t data[8];
} SlaveFeedbackRx_t;

volatile SlaveFeedbackStats_t slaveFeedbackStats;
static osMessageQueueId_t s_rx_queue;
static SlaveFeedbackSnapshot_t s_feedback;

// SlaveFeedback_StartCan：配置CAN2专用过滤器并启动接收通知
// 参数：无；CAN1和CAN2使用共同的14号过滤器分界
// 返回值：true表示已启动，false表示稍后由任务重试
static bool SlaveFeedback_StartCan(void)
{
    CAN_FilterTypeDef filter = {0};
    if (HAL_CAN_GetState(&hcan2) != HAL_CAN_STATE_LISTENING) {
        filter.FilterBank = 14U;
        filter.FilterMode = CAN_FILTERMODE_IDLIST;
        filter.FilterScale = CAN_FILTERSCALE_16BIT;
        filter.FilterIdHigh = FEEDBACK_BOARD1_ANGLE_ID << 5;
        filter.FilterIdLow = FEEDBACK_BOARD1_FORCE_ID << 5;
        filter.FilterMaskIdHigh = FEEDBACK_BOARD2_ANGLE_ID << 5;
        filter.FilterMaskIdLow = FEEDBACK_BOARD2_FORCE_ID << 5;
        filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter.FilterActivation = ENABLE;
        filter.SlaveStartFilterBank = 14U;
        if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK ||
            HAL_CAN_Start(&hcan2) != HAL_OK) return false;
    }
    return HAL_CAN_ActivateNotification(&hcan2,
               CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
}

// SlaveFeedback_Decode：按总线反馈协议校验并原子更新对应缓存
// 参数：frame - 原始CAN2帧，包含中断时间戳
// 返回值：true表示有效反馈已写入，false表示帧被拒绝
static bool SlaveFeedback_Decode(const SlaveFeedbackRx_t *frame)
{
    const CAN_RxHeaderTypeDef *header = &frame->header;
    if (header->IDE != CAN_ID_STD || header->RTR != CAN_RTR_DATA) return false;

    uint8_t board;
    bool angle_frame;
    switch (header->StdId) {
    case FEEDBACK_BOARD1_ANGLE_ID: board = 0U; angle_frame = true; break;
    case FEEDBACK_BOARD1_FORCE_ID: board = 0U; angle_frame = false; break;
    case FEEDBACK_BOARD2_ANGLE_ID: board = 1U; angle_frame = true; break;
    case FEEDBACK_BOARD2_FORCE_ID: board = 1U; angle_frame = false; break;
    default: return false;
    }
    uint32_t expected_length = angle_frame && board == 1U ? 7U : 8U;
    if (header->DLC != expected_length) return false;

    if (angle_frame) {
        uint16_t angles[3];
        for (uint8_t i = 0U; i < 3U; ++i) {
            angles[i] = ((uint16_t)frame->data[2U * i] << 8) |
                        frame->data[2U * i + 1U];
            if (angles[i] > 900U) return false;
        }
        uint8_t first = board == 0U ? 0U : 3U;
        taskENTER_CRITICAL();
        memcpy(&s_feedback.angle_tenths[first], angles, sizeof(angles));
        s_feedback.angle_valid_mask |= (uint8_t)(0x07U << first);
        s_feedback.angle_sequence[board] = frame->data[board == 0U ? 7U : 6U];
        s_feedback.angle_rx_ms[board] = frame->rx_ms;
        taskEXIT_CRITICAL();
    } else {
        uint8_t first = board == 0U ? 0U : 2U;
        uint8_t count = board == 0U ? 2U : 3U;
        uint8_t local_mask = board == 0U ? 0x03U : 0x07U;
        uint8_t global_mask = (uint8_t)(local_mask << first);
        taskENTER_CRITICAL();
        for (uint8_t i = 0U; i < count; ++i) {
            s_feedback.tangential_tenths[first + i] = frame->data[2U * i];
            s_feedback.normal_tenths[first + i] = frame->data[2U * i + 1U];
        }
        // 新帧的无效位立即替换旧位，不能让旧有效数据持续冒充本次数据。
        s_feedback.force_valid_mask =
            (s_feedback.force_valid_mask & (uint8_t)~global_mask) |
            (uint8_t)((frame->data[6] & local_mask) << first);
        s_feedback.force_sequence[board] = frame->data[7];
        s_feedback.force_rx_ms[board] = frame->rx_ms;
        taskEXIT_CRITICAL();
    }
    return true;
}

// SlaveFeedback_Task：消费CAN2反馈，并在启动失败后定期重试
// 参数：argument - 未使用
// 返回值：无，任务持续运行
static void SlaveFeedback_Task(void *argument)
{
    (void)argument;
    bool started = false;
    uint32_t retry_ms = HAL_GetTick() - 1000U;
    SlaveFeedbackRx_t frame;
    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        if (!started && (uint32_t)(now_ms - retry_ms) >= 1000U) {
            retry_ms = now_ms;
            started = SlaveFeedback_StartCan();
            if (!started) slaveFeedbackStats.start_failures++;
        }
        if (osMessageQueueGet(s_rx_queue, &frame, NULL, 20U) == osOK) {
            if (SlaveFeedback_Decode(&frame)) slaveFeedbackStats.accepted_frames++;
            else slaveFeedbackStats.rejected_frames++;
        }
    }
}

// SlaveFeedback_Init：创建独立反馈队列及普通优先级接收任务
// 参数：无
// 返回值：true表示全部软件资源创建成功
bool SlaveFeedback_Init(void)
{
    memset(&s_feedback, 0, sizeof(s_feedback));
    s_rx_queue = osMessageQueueNew(16U, sizeof(SlaveFeedbackRx_t), NULL);
    if (s_rx_queue == NULL) return false;
    const osThreadAttr_t attr = {
        .name = "SlaveFeedback", .stack_size = 1024U,
        .priority = osPriorityNormal
    };
    return osThreadNew(SlaveFeedback_Task, NULL, &attr) != NULL;
}

// SlaveFeedback_OnCanRx：从中断把完整帧复制入反馈队列
// 参数：header - 接收头；data - 八字节数据区；rx_ms - 接收时刻
// 返回值：无；尚未初始化或队列满时只记录丢帧
void SlaveFeedback_OnCanRx(const CAN_RxHeaderTypeDef *header,
                           const uint8_t data[8], uint32_t rx_ms)
{
    if (header == NULL || data == NULL) return;
    SlaveFeedbackRx_t frame = {.header = *header, .rx_ms = rx_ms};
    memcpy(frame.data, data, sizeof(frame.data));
    if (s_rx_queue == NULL ||
        osMessageQueuePut(s_rx_queue, &frame, 0U, 0U) != osOK) {
        slaveFeedbackStats.queue_drops++;
    }
}

// SlaveFeedback_GetSnapshot：复制缓存后独立判断四类反馈的新鲜度
// 参数：out - 输出快照；复制和计时在同一临界区完成，防止新帧时间晚于参考时刻
// 返回值：无；使用无符号差值兼容毫秒计数自然回绕
void SlaveFeedback_GetSnapshot(SlaveFeedbackSnapshot_t *out)
{
    if (out == NULL) return;
    taskENTER_CRITICAL();
    *out = s_feedback;
    uint32_t now_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
    for (uint8_t board = 0U; board < 2U; ++board) {
        if ((uint32_t)(now_ms - out->angle_rx_ms[board]) >= SLAVE_FEEDBACK_TIMEOUT_MS) {
            out->angle_valid_mask &= (uint8_t)~(board == 0U ? 0x07U : 0x38U);
        }
        if ((uint32_t)(now_ms - out->force_rx_ms[board]) >= SLAVE_FEEDBACK_TIMEOUT_MS) {
            out->force_valid_mask &= (uint8_t)~(board == 0U ? 0x03U : 0x1CU);
        }
    }
}
