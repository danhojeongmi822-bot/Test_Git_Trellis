/**
 * @file    bsp_can.h
 * @brief   CAN 总线板级支持包 (BSP) 接口声明
 *
 * 提供 CAN1 外设的初始化、数据发送及接收中断回调封装。
 * 仅依赖 HAL CAN 驱动，不包含任何应用层逻辑。
 *
 * @note    接收回调 HAL_CAN_RxFifo0MsgPendingCallback 在本模块内实现，
 *          会将原始帧数据转发给 Motor 层解析。
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "can.h"

/* ------------------------------------------------------------------ */
/*  数据结构                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief CAN 发送数据包
 *
 * 封装 HAL 发送头与 8 字节数据帧，以及 HAL 返回的邮箱编号。
 * CAN 协议规定数据帧最大 8 字节，tx_mailbox 用于查询发送状态。
 */
typedef struct
{
    CAN_TxHeaderTypeDef tx_header;  /**< HAL 发送帧头（ID、帧类型、DLC 等） */
    uint8_t             tx_data[8]; /**< 发送数据，固定 8 字节               */
    uint32_t            tx_mailbox; /**< HAL 返回的发送邮箱编号              */
} CAN_TxPacket_t;

/**
 * @brief CAN 接收数据包
 *
 * 封装 HAL 接收头与 8 字节数据帧，供中断回调使用。
 */
typedef struct
{
    CAN_RxHeaderTypeDef rx_header;  /**< HAL 接收帧头（ID、DLC 等）          */
    uint8_t             rx_data[8]; /**< 接收数据，固定 8 字节               */
} CAN_RxPacket_t;

/* ------------------------------------------------------------------ */
/*  接口函数                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief  初始化 CAN1 外设
 *
 * 配置接收滤波器（全通模式）、启动 CAN1 并使能 FIFO0 接收中断。
 * 必须在使用任何 CAN 收发功能前调用一次。
 */
void BSP_CAN_Init(void);

/**
 * @brief  通过指定 CAN 外设发送 8 字节标准帧
 *
 * @param  hcan    目标 CAN 外设句柄（如 &hcan1）
 * @param  std_id  11 位标准帧 ID（0x000 ~ 0x7FF）
 * @param  data    指向 8 字节发送缓冲区的指针
 */
void BSP_CAN_Send_Data(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t *data);

#endif /* BSP_CAN_H */