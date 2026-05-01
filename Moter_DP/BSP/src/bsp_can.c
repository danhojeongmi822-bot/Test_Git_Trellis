/**
 * @file    bsp_can.c
 * @brief   CAN 总线板级支持包 (BSP) 实现
 *
 * 实现 CAN1 滤波器配置、数据发送及 FIFO0 接收中断回调。
 * 接收到的帧直接转发给 Motor 层进行解析，本层不做业务处理。
 */

#include "bsp_can.h"
#include "Motor.h"

/* ------------------------------------------------------------------ */
/*  接口函数实现                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  初始化 CAN1 外设
 *
 * 滤波器配置为 32 位掩码模式，掩码全 0 表示接受所有 ID，
 * 接收报文路由到 FIFO0，并使能 FIFO0 非空中断。
 */
void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef filter_cfg = {0};

    filter_cfg.FilterActivation     = ENABLE;
    filter_cfg.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter_cfg.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter_cfg.FilterIdHigh         = 0x0000;
    filter_cfg.FilterIdLow          = 0x0000;
    filter_cfg.FilterMaskIdHigh     = 0x0000; /* 掩码全 0：接受所有 ID */
    filter_cfg.FilterMaskIdLow      = 0x0000;
    filter_cfg.FilterBank           = 0;
    filter_cfg.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1, &filter_cfg);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief  通过指定 CAN 外设发送 8 字节标准帧
 *
 * 固定 DLC = 8，使用标准帧格式（IDE = CAN_ID_STD）和数据帧（RTR = CAN_RTR_DATA）。
 *
 * @param  hcan    目标 CAN 外设句柄
 * @param  std_id  11 位标准帧 ID
 * @param  data    指向 8 字节发送缓冲区的指针
 */
void BSP_CAN_Send_Data(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t *data)
{
    CAN_TxPacket_t pkt;

    pkt.tx_header.StdId = std_id;
    pkt.tx_header.IDE   = CAN_ID_STD;
    pkt.tx_header.RTR   = CAN_RTR_DATA;
    pkt.tx_header.DLC   = 8;

    for (uint8_t i = 0; i < 8; i++)
    {
        pkt.tx_data[i] = data[i];
    }

    HAL_CAN_AddTxMessage(hcan, &pkt.tx_header, pkt.tx_data, &pkt.tx_mailbox);
}

/* ------------------------------------------------------------------ */
/*  HAL 回调函数                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  CAN FIFO0 接收中断回调（覆盖 HAL 弱定义）
 *
 * 从 FIFO0 读取一帧数据后，将帧 ID 和数据转发给 Motor 层解析。
 * 仅处理 CAN1 实例，其他实例的中断将被忽略。
 *
 * @param  hcan  触发中断的 CAN 外设句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxPacket_t pkt;

    if (hcan->Instance != CAN1)
    {
        return;
    }

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pkt.rx_header, pkt.rx_data);
    Motor_Process_CAN_Data(pkt.rx_header.StdId, pkt.rx_data);
}