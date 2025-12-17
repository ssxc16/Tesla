#include "motor.h"
#include "main.h"
#include "delay.h" // 如果有HAL_Delay或自定义delay
#include <stdint.h>

// 前进指定距离（单位：mm）
void Motor_Forward_Distance(uint16_t dist_mm)
{
    if(dist_mm == 0) return;
    uint32_t time_ms = (uint32_t)(dist_mm * K_FORWARD_TIME);
    MOTOR_FORWARD(MOTOR_SPEED);
    HAL_Delay(time_ms);
    MOTOR_STOP();
}

// 后退指定距离（单位：mm）
void Motor_Backward_Distance(float distance_m)
{
    if(distance_m <= 0) return;
    uint32_t time_ms = (uint32_t)(distance_m * K_FORWARD_TIME);
    MOTOR_BACK(MOTOR_SPEED);
    HAL_Delay(time_ms);
    MOTOR_STOP();
}

// 左转指定角度（度）
void Motor_Turn_Left(int16_t angle_deg)
{
    if(angle_deg == 0) return;
    uint32_t time_ms = (uint32_t)(angle_deg * K_TURN_TIME);
    MOTOR_SPINL(MOTOR_SPEED); // 左转
    HAL_Delay(time_ms);
    MOTOR_STOP();
}

// 右转指定角度（度）
void Motor_Turn_Right(int16_t angle_deg)
{
    if(angle_deg == 0) return;
    uint32_t time_ms = (uint32_t)(angle_deg * K_TURN_TIME);
    MOTOR_SPINR(MOTOR_SPEED); // 右转
    HAL_Delay(time_ms);
    MOTOR_STOP();
}

//=== 函数封装实现 ===
void Motor_SetSpeedLR(int speed_L, int speed_R)
{
    MOTOR_SET_SPD(speed_L, speed_R);
}

void Motor_Stop(void)
{
    MOTOR_STOP();
}