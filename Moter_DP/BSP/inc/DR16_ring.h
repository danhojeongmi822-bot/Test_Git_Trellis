/**
 * @file    DR16_ring.h
 * @brief   DT7/DR16 遥控器驱动接口声明
 *
 * 提供基于 USART3 + DMA + 环形缓冲区的 DBUS 协议接收与解析接口。
 *
 * 数据流：
 *   USART3(DMA) → DMA_Rx_Buffer
 *               → HAL_UARTEx_RxEventCallback（HAL 空闲事件）
 *               → remote_control_interrupt → gx_ring_DR16（环形缓冲区）
 *               → DR17_DT7_Proc → DBUS_data_frames → DR16_ctrl
 *               → DR16_GetData（供外部读取）
 *
 * @note    DR16_ctrl 直接暴露给外部，也可通过 DR16_GetData 获取指针。
 *          Remote_Control_Safety_Check 需在主循环中定期调用，
 *          超过 50ms 未收到数据则自动清零摇杆并置下线标志。
 */

#ifndef DR16_RING_H__
#define DR16_RING_H__

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  数据结构                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief DT7 遥控器完整数据包
 *
 * 包含摇杆通道、拨杆、鼠标和键盘四个子结构。
 * 通道值范围：364 ~ 1684，中位值约 1024。
 * 拨杆值：1 = 上，2 = 中，3 = 下。
 */
typedef struct
{
    struct
    {
        uint16_t ch0;    /**< 右摇杆左右轴 */
        uint16_t ch1;    /**< 右摇杆上下轴 */
        uint16_t ch2;    /**< 左摇杆左右轴 */
        uint16_t ch3;    /**< 左摇杆上下轴 */
        uint8_t  s1;     /**< 左拨杆（1=上 2=中 3=下） */
        uint8_t  s2;     /**< 右拨杆（1=上 2=中 3=下） */
    } remote;

    struct
    {
        int16_t x;       /**< 鼠标 X 轴增量 */
        int16_t y;       /**< 鼠标 Y 轴增量 */
        int16_t z;       /**< 鼠标滚轮增量  */
        uint8_t press_l; /**< 左键按下状态（1=按下） */
        uint8_t press_r; /**< 右键按下状态（1=按下） */
    } mouse;

    struct
    {
        uint16_t back;   /**< 键盘按键位域（每位对应一个按键） */
    } keyboard;

    uint32_t last_Tick;   /**< 最后一次成功解析的系统时间戳（HAL_GetTick，ms） */
    uint8_t  online_Flag; /**< 在线标志：1 = 在线，0 = 超时离线                 */

} DBUS_Data_t;

/* ------------------------------------------------------------------ */
/*  全局变量声明                                                        */
/* ------------------------------------------------------------------ */

/** 遥控器解析结果，由 DBUS_data_frames 更新，外部只读访问 */
extern DBUS_Data_t DR16_ctrl;

/* ------------------------------------------------------------------ */
/*  接口函数                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief  初始化 DR16 驱动，启动 USART3 DMA 接收
 *
 * 初始化环形缓冲区，以 ReceiveToIdle_DMA 模式启动 USART3 DMA 接收，
 * 并禁用 DMA 半满中断（避免半包数据触发回调导致解析错误）。
 * 上电时调用一次，UART 错误恢复时也会重新调用。
 */
void DR16_DT7_Init(void);

/**
 * @brief  遥控器数据处理任务（主循环轮询调用）
 *
 * 从环形缓冲区中取出完整的 DBUS 帧（18 字节）并调用 DBUS_data_frames 解析。
 * 若缓冲区中有多帧积压，会循环取出直到不足一帧为止，只解析最后一帧。
 *
 * @note   需在主循环中以足够高的频率调用，避免环形缓冲区溢出。
 */
void DR17_DT7_Proc(void);

/**
 * @brief  解析 18 字节 DBUS 原始帧，结果写入 DR16_ctrl
 *
 * DBUS 位域布局（每通道 11 bit，小端序）：
 *   Byte  0       : ch0[7:0]
 *   Byte  1[2:0]  : ch0[10:8]，Byte 1[7:3] : ch1[4:0]
 *   Byte  2[5:0]  : ch1[10:5]，Byte 2[7:6] : ch2[1:0]
 *   Byte  3       : ch2[9:2]
 *   Byte  4[0]    : ch2[10]，  Byte 4[7:1] : ch3[6:0]
 *   Byte  5[3:0]  : ch3[10:7]，Byte 5[5:4] : s2，Byte 5[7:6] : s1
 *   Byte  6~13    : 鼠标 x/y/z/左键/右键
 *   Byte  14~15   : 键盘位域
 *
 * @param  DBUF_frames  指向 18 字节原始帧数据的指针
 */
void DBUS_data_frames(const uint8_t *DBUF_frames);

/**
 * @brief  遥控器在线安全检查（主循环轮询调用）
 *
 * 若距上次成功解析超过 50ms，则将 online_Flag 置 0 并清零所有摇杆通道，
 * 防止遥控器断连后电机继续执行最后一次指令。
 */
void Remote_Control_Safety_Check(void);

/**
 * @brief  DMA 接收中断处理（由 HAL_UARTEx_RxEventCallback 调用）
 *
 * 将 DMA 缓冲区中本次接收到的数据压入环形缓冲区，
 * 然后重新启动 DMA 接收，准备接收下一帧。
 *
 * @param  Size  本次 DMA 实际接收的字节数（由 HAL 传入）
 */
void remote_control_interrupt(uint16_t Size);

#endif /* DR16_RING_H__ */