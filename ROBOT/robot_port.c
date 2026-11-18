/**
 * @file robot_port.c
 * @brief HAL → ECF BSP 的端口适配层实现
 */
#include "robot_port.h"

#include <string.h>

#include "fdcan.h"
#include "cod-fdcan.h"

/* ======================== 内部辅助 ======================== */

static FDCAN_HandleTypeDef *port_handle(void *handle)
{
    return (FDCAN_HandleTypeDef *)handle;
}

/* ======================== driver ops ======================== */

static bsp_status_t port_init(void *handle)
{
    /* 外设已由 COD 的 BSP_FDCAN_Init() 完成初始化，此处仅校验句柄 */
    return (port_handle(handle) != NULL) ? BSP_STATUS_OK : BSP_STATUS_INVALID_ARGUMENT;
}

static bsp_status_t port_deinit(void *handle)
{
    FDCAN_HandleTypeDef *hfdcan = port_handle(handle);
    if (hfdcan == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    (void)HAL_FDCAN_Stop(hfdcan);
    return BSP_STATUS_OK;
}

static bsp_status_t port_start(void *handle)
{
    FDCAN_HandleTypeDef *hfdcan = port_handle(handle);
    if (hfdcan == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    /* BSP_FDCAN_Init() 已启动过，重复 Start 视为成功 */
    if (hfdcan->State == HAL_FDCAN_STATE_BUSY) {
        return BSP_STATUS_OK;
    }
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return BSP_STATUS_IO_ERROR;
    }
    return BSP_STATUS_OK;
}

static bsp_status_t port_stop(void *handle)
{
    FDCAN_HandleTypeDef *hfdcan = port_handle(handle);
    if (hfdcan == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    (void)HAL_FDCAN_Stop(hfdcan);
    return BSP_STATUS_OK;
}

static bsp_status_t port_configure_filter(void *handle, const bsp_can_filter_t *filter)
{
    FDCAN_HandleTypeDef *hfdcan = port_handle(handle);
    FDCAN_FilterTypeDef hw_filter = {0};

    if (hfdcan == NULL || filter == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    hw_filter.IdType = (filter->id_type == BSP_CAN_ID_EXTENDED) ? FDCAN_EXTENDED_ID
                                                               : FDCAN_STANDARD_ID;
    hw_filter.FilterIndex = filter->filter_index;
    hw_filter.FilterType = FDCAN_FILTER_MASK;
    hw_filter.FilterConfig = (filter->receive_fifo == BSP_CAN_RX_FIFO_1)
                                 ? FDCAN_FILTER_TO_RXFIFO1
                                 : FDCAN_FILTER_TO_RXFIFO0;
    hw_filter.FilterID1 = filter->identifier;
    hw_filter.FilterID2 = filter->mask;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &hw_filter) != HAL_OK) {
        return BSP_STATUS_IO_ERROR;
    }
    return BSP_STATUS_OK;
}

static bsp_status_t port_transmit(void *handle,
                                  const bsp_can_frame_t *frame,
                                  uint32_t timeout_ms)
{
    FDCAN_HandleTypeDef *hfdcan = port_handle(handle);
    FDCAN_TxHeaderTypeDef header = {0};

    (void)timeout_ms;

    if (hfdcan == NULL || frame == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (frame->data_length > 8U) {
        return BSP_STATUS_OUT_OF_RANGE;
    }

    header.Identifier = frame->identifier;
    header.IdType = (frame->id_type == BSP_CAN_ID_EXTENDED) ? FDCAN_EXTENDED_ID
                                                           : FDCAN_STANDARD_ID;
    header.TxFrameType = (frame->frame_type == BSP_CAN_FRAME_REMOTE) ? FDCAN_REMOTE_FRAME
                                                                    : FDCAN_DATA_FRAME;
    header.DataLength = (uint32_t)frame->data_length;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)frame->data) != HAL_OK) {
        return BSP_STATUS_IO_ERROR;
    }
    return BSP_STATUS_OK;
}

static bsp_status_t port_receive(void *handle,
                                 bsp_can_receive_fifo_t fifo,
                                 bsp_can_frame_t *frame)
{
    FDCAN_HandleTypeDef *hfdcan = port_handle(handle);
    FDCAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint32_t hw_fifo;
    uint8_t length;

    if (hfdcan == NULL || frame == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    hw_fifo = (fifo == BSP_CAN_RX_FIFO_1) ? FDCAN_RX_FIFO1 : FDCAN_RX_FIFO0;

    if (HAL_FDCAN_GetRxMessage(hfdcan, hw_fifo, &header, data) != HAL_OK) {
        return BSP_STATUS_IO_ERROR;
    }

    frame->identifier = header.Identifier;
    frame->id_type = (header.IdType == FDCAN_EXTENDED_ID) ? BSP_CAN_ID_EXTENDED
                                                         : BSP_CAN_ID_STANDARD;
    frame->frame_type = (header.RxFrameType == FDCAN_REMOTE_FRAME) ? BSP_CAN_FRAME_REMOTE
                                                                  : BSP_CAN_FRAME_DATA;

    /* HAL 返回的是 DLC 编码（0~15），经典 CAN 只取前 8 字节 */
    length = (uint8_t)header.DataLength;
    if (length > 8U) {
        length = 8U;
    }
    frame->data_length = length;
    (void)memcpy(frame->data, data, length);

    return BSP_STATUS_OK;
}

static bsp_status_t port_get_tx_free_level(const void *handle, uint32_t *free_level)
{
    const FDCAN_HandleTypeDef *hfdcan = (const FDCAN_HandleTypeDef *)handle;

    if (hfdcan == NULL || free_level == NULL) {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    /* H7 FDCAN 无独立发送邮箱计数，这里用 TXFQS 的 TFFL 空闲等级近似 */
    *free_level = (uint32_t)((hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFFL) >>
                             FDCAN_TXFQS_TFFL_Pos);
    return BSP_STATUS_OK;
}

static const bsp_can_driver_ops_t k_fdcan_ops = {
    .init = port_init,
    .deinit = port_deinit,
    .start = port_start,
    .stop = port_stop,
    .configure_filter = port_configure_filter,
    .transmit = port_transmit,
    .receive = port_receive,
    .get_tx_free_level = port_get_tx_free_level,
};

const bsp_can_driver_ops_t *robot_port_fdcan_ops(void)
{
    return &k_fdcan_ops;
}
