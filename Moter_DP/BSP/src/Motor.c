/**
 * @file    Motor.c
 * @brief   RM3508 电机数据解析实现
 *
 * 维护底盘四个电机的状态数组，并提供 CAN 反馈帧解析函数。
 * 本模块只负责数据层，不包含控制逻辑或测试代码。
 */

#include "Motor.h"

#include "stm32f4xx_hal.h"


/*              初始化电机接口                              */
void Motor_Init(void)
{
    PID_InitTypeDef pid_init;
    pid_init.Kp = 10.01f;
    pid_init.Ki = 0.01f;
    pid_init.Kd = 0.01f;
    pid_init.Setpoint = 0.00f;
    //设置电流环电流输入输出最低上下限
    pid_init.OutputUpperLimit = 16384.0f;
    pid_init.OutputLowerLimit = -16384.0f;
    // 默认输出值，在系统启动或复位时的初始输出值
    pid_init.DefaultOutput = 0.0f;

    //初始化底盘四个电机的PID引擎
    for (int i = 0;i < CHASSIS_MOTOR_COUNT;i++)
    {
        PID_Init(&chassis_motors[i].Speed_Pid,&pid_init);
        chassis_motors[i].target_Speed = 0;
        chassis_motors[i].target_current = 0;
    }

}

void Motor_Speed_Loop(void)
{
    uint64_t cot_time_us = (uint64_t)HAL_GetTick()*1000;

    for (int i = 0;i<CHASSIS_MOTOR_COUNT;i++)
    {
        //设定目标速度 更新PID设定值
        PID_ChangeSetpoint(&chassis_motors[i].Speed_Pid,chassis_motors[i].target_Speed);

        float out_current = PID_Compute1(&chassis_motors[i].Speed_Pid,chassis_motors[i].target_Speed,cot_time_us);

        chassis_motors[i].target_current = (int16_t)out_current;

    }
    test_Can_3508open_loop(chassis_motors[0].target_current,
                           chassis_motors[1].target_current,
                           chassis_motors[2].target_current,
                           chassis_motors[3].target_current);


}




/* ------------------------------------------------------------------ */
/*  全局变量定义                                                        */
/* ------------------------------------------------------------------ */

/** 底盘四个电机对象，下标 0~3 对应 CAN ID 0x201~0x204 */
Motor_t chassis_motors[CHASSIS_MOTOR_COUNT] = {0};

/* ------------------------------------------------------------------ */
/*  接口函数实现                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  解析 CAN 反馈帧并更新对应电机的测量数据
 *
 * RM3508 反馈帧格式（8 字节）：
 *   Byte 0~1 : 机械角度（大端，0~8191）
 *   Byte 2~3 : 转子转速（大端，有符号，RPM）
 *   Byte 4~5 : 实际电流（大端，有符号，mA）
 *   Byte 6   : 电机温度（°C）
 *   Byte 7   : 保留
 *
 * @param  can_id  接收帧的标准 CAN ID
 * @param  data    指向 8 字节接收数据缓冲区的指针
 */
void Motor_Process_CAN_Data(uint32_t can_id, uint8_t *data)
{
    if (can_id < MOTOR_CAN_ID_BASE || can_id > MOTOR_CAN_ID_MAX)
    {
        return;
    }

    uint8_t idx = (uint8_t)(can_id - MOTOR_CAN_ID_BASE);

    chassis_motors[idx].measure.mechanical_angle = (uint16_t)((data[0] << 8) | data[1]);
    chassis_motors[idx].measure.rotor_speed      = (int16_t) ((data[2] << 8) | data[3]);
    chassis_motors[idx].measure.electricity      = (int16_t) ((data[4] << 8) | data[5]);
    chassis_motors[idx].measure.temperature      = data[6];
}