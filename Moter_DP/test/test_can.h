/**
 * @file    test_can.h
 * @brief   CAN 通信测试接口声明
 *
 * 提供针对 RM3508 电机的开环测试函数，
 * 仅用于调试阶段，不应在正式工程中调用。
 */

#ifndef TEST_CAN_H
#define TEST_CAN_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  接口函数                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief  以开环方式向四个底盘电机发送指定电流值
 *
 * 将四路电流值打包为 CAN 帧，通过 CAN ID 0x200 发送给 RM3508 电调。
 * 电流值范围：-16384 ~ +16384（对应 RM3508 最大电流）。
 *
 * @param  motor_1  电机 1（CAN ID 0x201）目标电流
 * @param  motor_2  电机 2（CAN ID 0x202）目标电流
 * @param  motor_3  电机 3（CAN ID 0x203）目标电流
 * @param  motor_4  电机 4（CAN ID 0x204）目标电流
 */
void test_Can_3508open_loop(uint16_t motor_1, uint16_t motor_2,
                            uint16_t motor_3, uint16_t motor_4);

/**
 * @brief  CAN 通信回环测试
 *
 * 持续向四个电机发送固定电流，并通过 UART6 打印电机 1 的反馈数据，
 * 用于验证 CAN 收发链路是否正常。
 *
 * @note   此函数包含死循环，调用后不会返回，仅用于上电调试。
 */
void Test_CAN_Loopback(void);

#endif /* TEST_CAN_H */