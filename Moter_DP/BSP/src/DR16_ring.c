/**
 * @file    DR16_ring.c
 * @brief   DT7/DR16 遥控器驱动实现
 *
 * 实现基于 USART3 + DMA + 环形缓冲区的 DBUS 协议接收与解析。
 *
 * 数据流：
 *   USART3(DMA) → DMA_Rx_Buffer
 *               → HAL_UARTEx_RxEventCallback（HAL 空闲事件，在 bsp_Uart.c 中分发）
 *               → remote_control_interrupt → gx_ring_DR16（环形缓冲区）
 *               → DR17_DT7_Proc → DBUS_data_frames → DR16_ctrl
 *
 * @note    使用 HAL_UARTEx_ReceiveToIdle_DMA 而非手动空闲中断，
 *          HAL 自动管理空闲检测并在回调中传入实际接收长度，
 *          无需手动读取 DMA 计数器。
 */

#include "stm32f4xx_hal.h"
#include "ringbuffer.h"
#include "usart.h"
#include "DR16_ring.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  常量定义                                                            */
/* ------------------------------------------------------------------ */

#define DBUS_DMA_BUF_SIZE   36U  /**< DMA 硬件接收缓冲区大小（字节），留余量防溢出 */
#define DBUS_RING_BUF_SIZE 128U  /**< 软件环形缓冲区大小（字节）                   */
#define DBUS_FRAME_SIZE     18U  /**< DBUS 标准帧长（字节）                        */

/* ------------------------------------------------------------------ */
/*  全局变量定义                                                        */
/* ------------------------------------------------------------------ */

/** 遥控器解析结果，外部通过 extern 只读访问 */
DBUS_Data_t DR16_ctrl;

/* ------------------------------------------------------------------ */
/*  模块内部变量                                                        */
/* ------------------------------------------------------------------ */

/** DMA 硬件接收缓冲区（HAL 直接搬运到这里） */
static uint8_t DMA_Rx_Buffer[DBUS_DMA_BUF_SIZE];

/** 环形缓冲区内存池 */
static uint8_t Ring_Rx_Buffer[DBUS_RING_BUF_SIZE];

/** 环形缓冲区控制块 */
struct rt_ringbuffer gx_ring_DR16;

/* ------------------------------------------------------------------ */
/*  接口函数实现                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  初始化 DR16 驱动，启动 USART3 DMA 接收
 *
 * 使用 HAL_UARTEx_ReceiveToIdle_DMA，HAL 内部自动管理空闲中断，
 * 在检测到空闲事件时触发 HAL_UARTEx_RxEventCallback 并传入实际接收长度。
 * 禁用 DMA 半满中断，避免半包数据触发回调导致解析错误。
 */
void DR16_DT7_Init(void)
{
    rt_ringbuffer_init(&gx_ring_DR16, Ring_Rx_Buffer, sizeof(Ring_Rx_Buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, DMA_Rx_Buffer, sizeof(DMA_Rx_Buffer));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT); /* 禁用半满中断，只在完整帧或空闲时触发 */
}

/**
 * @brief  DMA 接收中断处理（由 HAL_UARTEx_RxEventCallback 调用）
 *
 * 将本次 DMA 接收到的数据压入环形缓冲区，然后重新启动 DMA 接收。
 * 重新启动后同样禁用半满中断，保持与初始化时一致的行为。
 *
 * @param  Size  本次 DMA 实际接收的字节数（由 HAL 计算并传入）
 */
void remote_control_interrupt(uint16_t Size)
{
    /* 将本次接收到的数据压入环形缓冲区 */
    rt_ringbuffer_put(&gx_ring_DR16, DMA_Rx_Buffer, Size);

    /* 重新启动 DMA 接收，准备接收下一帧 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, DMA_Rx_Buffer, sizeof(DMA_Rx_Buffer));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

/**
 * @brief  遥控器数据处理任务（主循环轮询调用）
 *
 * 从环形缓冲区中循环取出完整的 DBUS 帧（18 字节）并解析。
 * 若缓冲区中积压了多帧，会丢弃旧帧只保留最新一帧，
 * 确保控制指令始终是最新的。
 */
void DR17_DT7_Proc(void)
{
    uint8_t frame_buf[DBUS_FRAME_SIZE];

    /* 循环取帧，丢弃旧帧，只解析最后一帧 */
    while (rt_ringbuffer_data_len(&gx_ring_DR16) >= DBUS_FRAME_SIZE)
    {
        rt_ringbuffer_get(&gx_ring_DR16, frame_buf, DBUS_FRAME_SIZE);
    }

    DBUS_data_frames(frame_buf);
}

/**
 * @brief  解析 18 字节 DBUS 原始帧，结果写入 DR16_ctrl
 *
 * DBUS 协议使用小端序位域打包，每个摇杆通道占 11 bit：
 *   ch0 = Byte[0] | (Byte[1] << 8)，取低 11 bit
 *   ch1 = (Byte[1] >> 3) | (Byte[2] << 5)，取低 11 bit
 *   ch2 = (Byte[2] >> 6) | (Byte[3] << 2) | (Byte[4] << 10)，取低 11 bit
 *   ch3 = (Byte[4] >> 1) | (Byte[5] << 7)，取低 11 bit
 *   s1  = Byte[5][7:6]，s2 = Byte[5][5:4]
 *   鼠标 x/y/z 各 16 bit 小端，按键各 1 字节
 *   键盘位域 Byte[14]
 *
 * @param  DBUF_frames  指向 18 字节原始帧数据的指针
 */
void DBUS_data_frames(const uint8_t *DBUF_frames)
{
    if (DBUF_frames == NULL)
        return;

    /* 摇杆通道（各 11 bit，小端位域解包） */
    DR16_ctrl.remote.ch0 = ((int16_t)DBUF_frames[0] | ((int16_t)DBUF_frames[1] << 8)) & 0x07FF;
    DR16_ctrl.remote.ch1 = (((int16_t)DBUF_frames[1] >> 3) | ((int16_t)DBUF_frames[2] << 5)) & 0x07FF;
    DR16_ctrl.remote.ch2 = (((int16_t)DBUF_frames[2] >> 6) | ((int16_t)DBUF_frames[3] << 2) |
                             ((int16_t)DBUF_frames[4] << 10)) & 0x07FF;
    DR16_ctrl.remote.ch3 = (((int16_t)DBUF_frames[4] >> 1) | ((int16_t)DBUF_frames[5] << 7)) & 0x07FF;

    /* 拨杆（各 2 bit，从 Byte[5] 高位提取） */
    DR16_ctrl.remote.s1 = ((DBUF_frames[5] >> 4) & 0x0C) >> 2;
    DR16_ctrl.remote.s2 =  (DBUF_frames[5] >> 4) & 0x03;

    /* 鼠标（各轴 16 bit 小端有符号整数） */
    DR16_ctrl.mouse.x       = (int16_t)((uint16_t)DBUF_frames[6]  | ((uint16_t)DBUF_frames[7]  << 8));
    DR16_ctrl.mouse.y       = (int16_t)((uint16_t)DBUF_frames[8]  | ((uint16_t)DBUF_frames[9]  << 8));
    DR16_ctrl.mouse.z       = (int16_t)((uint16_t)DBUF_frames[10] | ((uint16_t)DBUF_frames[11] << 8));
    DR16_ctrl.mouse.press_l = DBUF_frames[12];
    DR16_ctrl.mouse.press_r = DBUF_frames[13];

    /* 键盘位域 */
    DR16_ctrl.keyboard.back = (uint16_t)DBUF_frames[14];

    /* 更新时间戳和在线标志 */
    DR16_ctrl.last_Tick   = HAL_GetTick();
    DR16_ctrl.online_Flag = 1;
}

/**
 * @brief  遥控器在线安全检查（主循环轮询调用）
 *
 * 若距上次成功解析超过 50ms，则将 online_Flag 置 0 并清零所有摇杆通道，
 * 防止遥控器断连后电机继续执行最后一次指令造成失控。
 */
void Remote_Control_Safety_Check(void)
{
    if (HAL_GetTick() - DR16_ctrl.last_Tick > 50)
    {
        DR16_ctrl.online_Flag = 0;
        memset(&DR16_ctrl.remote, 0, sizeof(DR16_ctrl.remote));
    }
}