/**
 * @file    bsp_Uart.c
 * @brief   UART 板级支持包 (BSP) 实现
 *
 * 本模块职责：
 *   - 发送：DMA 异步发送（my_printf 调试打印 / BSP_UART_Send_DMA 原始数据流）
 *   - HAL 回调：统一接收事件分发入口，各外设的接收处理由应用层自行注册
 *
 * HAL 回调分发策略：
 *   HAL_UARTEx_RxEventCallback 是全局唯一的 HAL 接收事件回调，
 *   在此处按外设实例分发给各自的处理函数：
 *     USART3 → remote_control_interrupt（DR16 遥控器）
 *     其他外设 → 应用层在此处扩展
 *
 *   HAL_UART_ErrorCallback 处理 DMA 溢出等错误，
 *   对 USART3 执行停止 DMA → 清除溢出标志 → 重新初始化的恢复流程。
 */

#include "bsp_Uart.h"
#include "DR16_ring.h"

/* ------------------------------------------------------------------ */
/*  静态缓冲区                                                          */
/* ------------------------------------------------------------------ */

/** my_printf 专属发送缓冲区（避免与业务数据冲突） */
static uint8_t uart_tx_buf[UART_TX_BUF_SIZE];

/* ------------------------------------------------------------------ */
/*  接口函数实现                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  格式化输出到指定 UART 外设（调试用）
 *
 * 工作流程：
 *   1. vsnprintf 将格式化字符串写入静态缓冲区
 *   2. 轮询等待上次 DMA 发送完成
 *   3. 启动 DMA 异步发送
 *
 * @note   使用静态缓冲区，非线程安全，仅适用于单线程调试场景。
 */
int my_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
    static va_list ap;
    uint16_t len;

    va_start(ap, format);
    len = vsnprintf((char *)uart_tx_buf, UART_TX_BUF_SIZE, format, ap);
    va_end(ap);

    /* 等待上次 DMA 发送完成，避免覆盖正在发送的数据 */
    while (huart->gState == HAL_UART_STATE_BUSY_TX)
        ;

    HAL_UART_Transmit_DMA(huart, uart_tx_buf, len);
    return len;
}

/**
 * @brief  通过 DMA 异步发送原始字节流
 *
 * 等待上次 DMA 发送完成后再启动新一次传输，适用于发送协议帧等原始数据。
 */
void BSP_UART_Send_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    /* 确保上次 DMA 发送已完成，避免覆盖正在发送的数据 */
    while (huart->gState == HAL_UART_STATE_BUSY_TX)
        ;

    HAL_UART_Transmit_DMA(huart, data, len);
}

/* ------------------------------------------------------------------ */
/*  HAL 回调函数（覆盖 HAL 弱定义）                                    */
/* ------------------------------------------------------------------ */

/**
 * @brief  UART DMA 接收事件回调（覆盖 HAL 弱定义）
 *
 * 由 HAL 在以下事件发生时调用：
 *   - DMA 搬运完成（DMA 计数器归零）
 *   - UART 检测到空闲（一帧数据接收完毕）
 *
 * 按外设实例分发给各自的处理函数：
 *   USART3 → remote_control_interrupt（DR16 遥控器解析）
 *   其他外设 → 应用层在此处按需扩展
 *
 * @param  huart  触发事件的 UART 外设句柄
 * @param  Size   本次实际接收的字节数（由 HAL 计算）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        remote_control_interrupt(Size);
    }
}

/**
 * @brief  UART 错误回调（覆盖 HAL 弱定义）
 *
 * 处理 DMA 接收过程中的硬件错误（如 ORE 溢出错误）。
 * 对 USART3 执行恢复流程：停止 DMA → 清除溢出标志 → 重新初始化。
 *
 * @note   ORE（Overrun Error）在 DMA 来不及搬运时触发，
 *         重新初始化可恢复正常接收，不会丢失后续数据。
 *
 * @param  huart  发生错误的 UART 外设句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HAL_UART_DMAStop(huart);
        __HAL_UART_CLEAR_OREFLAG(huart); /* 清除溢出错误标志，否则 UART 会持续报错 */
        DR16_DT7_Init();                 /* 重新启动 DMA 接收，恢复正常工作 */
    }
}