/**
 * @file    bsp_Uart.h
 * @brief   UART 板级支持包 (BSP) 接口声明
 *
 * 本模块只负责发送，接收处理由应用层自行实现：
 *   - 发送：DMA 异步发送，等待上次发送完成后再启动新一次传输
 *   - 调试：my_printf 封装格式化打印，内部使用独立的静态发送缓冲区
 *
 * @note    HAL_UARTEx_RxEventCallback 在 bsp_Uart.c 中统一实现，
 *          应用层如需接入新的 UART 接收处理，在该回调中按外设实例扩展即可。
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdarg.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

/* ------------------------------------------------------------------ */
/*  容量配置                                                            */
/* ------------------------------------------------------------------ */

#define UART_TX_BUF_SIZE   256U  /**< my_printf 专属发送缓冲区容量 */

/* ------------------------------------------------------------------ */
/*  接口函数                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief  通过 DMA 异步发送原始字节流
 *
 * 等待上次 DMA 发送完成后再启动新一次传输，避免覆盖正在发送的数据。
 *
 * @param  huart  目标 UART 外设句柄
 * @param  data   指向待发送数据缓冲区的指针
 * @param  len    待发送字节数
 */
void BSP_UART_Send_DMA(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);

/**
 * @brief  格式化输出到指定 UART 外设（调试用）
 *
 * 内部使用静态缓冲区（UART_TX_BUF_SIZE 字节），超出部分截断。
 * 等待上次 DMA 发送完成后再发送，适用于低频调试打印。
 *
 * @param  huart   目标 UART 外设句柄（如 &huart6）
 * @param  format  printf 风格格式字符串
 * @param  ...     可变参数列表
 * @return 实际写入缓冲区的字节数（不含终止符）
 */
int my_printf(UART_HandleTypeDef *huart, const char *format, ...);

#endif /* BSP_UART_H */