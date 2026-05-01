/**
 * @file    Motor.h
 * @brief   RM3508 电机数据结构与接口声明
 *
 * 定义底盘电机的测量数据结构体和电机对象，
 * 提供 CAN 原始数据解析接口。
 *
 * @note    chassis_motors 数组由本模块维护，其他模块通过 extern 只读访问。
 *          电机 ID 映射：CAN ID 0x201~0x204 对应 chassis_motors[0]~[3]。
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "PID.h"
#include "test_can.h"
/* ------------------------------------------------------------------ */
/*  常量定义                                                            */
/* ------------------------------------------------------------------ */

#define CHASSIS_MOTOR_COUNT  4U   /**< 底盘电机数量                        */

#define MOTOR_CAN_ID_BASE    0x201U /**< 底盘电机 CAN ID 起始值（含）       */
#define MOTOR_CAN_ID_MAX     0x204U /**< 底盘电机 CAN ID 结束值（含）       */

/* ------------------------------------------------------------------ */
/*  数据结构                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 电机实时测量数据
 *
 * 由 CAN 反馈帧（8 字节）解析得到，每次收到反馈帧后更新。
 */
typedef struct
{
    uint16_t mechanical_angle; /**< 机械角度，范围 0~8191，对应 0~360°     */
    int16_t  rotor_speed;      /**< 转子转速，单位 RPM                      */
    int16_t  electricity;      /**< 实际电流，单位 mA（原始值需乘以系数）   */
    uint8_t  temperature;      /**< 电机温度，单位 °C                       */
} Motor_Measure_t;

/**
 * @brief 单个电机对象
 *
 * 包含实时测量数据和控制层写入的目标电流值。
 */
typedef struct
{
    Motor_Measure_t measure;       /**< 电机实时反馈数据                    */
    int16_t         target_Speed; /**   目标转速   **/
    int16_t         target_current;/**< 控制层设定的目标电流，单位 mA       */
    PID_TypeDef     Speed_Pid;   /**速度环PID控制电流输出**/
} Motor_t;

/*               初始化底盘电机接口                          */
void Motor_Init(void);

/* ------------------------------------------------------------------ */
/*  全局变量声明                                                        */
/* ------------------------------------------------------------------ */

/** 底盘四个电机对象数组，下标 0~3 对应 CAN ID 0x201~0x204 */
extern Motor_t chassis_motors[CHASSIS_MOTOR_COUNT];

/* ------------------------------------------------------------------ */
/*  接口函数                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief  解析 CAN 反馈帧并更新对应电机的测量数据
 *
 * 仅处理 CAN ID 在 [MOTOR_CAN_ID_BASE, MOTOR_CAN_ID_MAX] 范围内的帧，
 * 其余 ID 的帧将被忽略。
 *
 * @param  can_id  接收帧的标准 CAN ID
 * @param  data    指向 8 字节接收数据缓冲区的指针
 */
void Motor_Process_CAN_Data(uint32_t can_id, uint8_t *data);

#endif /* MOTOR_H */