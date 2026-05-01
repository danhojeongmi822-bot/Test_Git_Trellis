/**
 * @file    test_can.c
 * @brief   CAN 通信测试实现
 *
 * 包含 RM3508 开环控制测试和 CAN 回环通信验证，
 * 仅用于调试阶段，不应集成到正式工程的主循环中。
 */

#include "test_can.h"
#include "bsp_can.h"
#include "bsp_Uart.h"
#include "Motor.h"
#include "usart.h"

/* ------------------------------------------------------------------ */
/*  接口函数实现                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  以开环方式向四个底盘电机发送指定电流值
 *
 * RM3508 电调控制帧格式（CAN ID 0x200，8 字节）：
 *   Byte 0~1 : 电机 1 电流（大端）
 *   Byte 2~3 : 电机 2 电流（大端）
 *   Byte 4~5 : 电机 3 电流（大端）
 *   Byte 6~7 : 电机 4 电流（大端）
 */
void test_Can_3508open_loop(uint16_t motor_1, uint16_t motor_2,
                            uint16_t motor_3, uint16_t motor_4)
{
    uint8_t tx_buf[8];

    tx_buf[0] = (uint8_t)((motor_1 >> 8) & 0xFF);
    tx_buf[1] = (uint8_t)(motor_1 & 0xFF);
    tx_buf[2] = (uint8_t)((motor_2 >> 8) & 0xFF);
    tx_buf[3] = (uint8_t)(motor_2 & 0xFF);
    tx_buf[4] = (uint8_t)((motor_3 >> 8) & 0xFF);
    tx_buf[5] = (uint8_t)(motor_3 & 0xFF);
    tx_buf[6] = (uint8_t)((motor_4 >> 8) & 0xFF);
    tx_buf[7] = (uint8_t)(motor_4 & 0xFF);

    BSP_CAN_Send_Data(&hcan1, 0x200, tx_buf);
}

/**
 * @brief  CAN 通信回环测试
 *
 * 持续给四个电机发送 1000 的电流值，并通过 UART6 打印电机 1（索引 0）
 * 的机械角度、电流、转速和温度，用于验证 CAN 收发链路。
 *
 * @note   包含死循环，仅用于上电调试，调用后不会返回。
 */
void Test_CAN_Loopback(void)
{
    BSP_CAN_Init();

    while (1)
    {
        /* 向四个电机发送 1000 的开环电流 */
        test_Can_3508open_loop(1000, 1000, 1000, 1000);

        /* 打印电机 1（索引 0）的反馈数据 */
        my_printf(&huart6, "angle=%u, current=%d, speed=%d, temp=%u\r\n",
                  chassis_motors[0].measure.mechanical_angle,
                  chassis_motors[0].measure.electricity,
                  chassis_motors[0].measure.rotor_speed,
                  chassis_motors[0].measure.temperature);
    }
}