#include "motor.h"
#include "main.h"
#include "delay.h" // 如果有HAL_Delay或自定义delay

// 需根据实际测试调整
#define MOTOR_SPEED      60      // 电机PWM占空比（0~100）
#define K_FORWARD_TIME   3.5f   // 需要标定：每毫米多少ms（示例）
#define K_TURN_TIME      7.5f   // 转动1度所需时间(ms)

// 前进指定距离（单位：米）
void Motor_Forward_Distance(uint16_t dist_mm)
{
    if(dist_mm == 0) return;
    uint32_t time_ms = (uint32_t)(dist_mm * K_FORWARD_TIME);
    MOTOR_FORWARD(MOTOR_SPEED);
    HAL_Delay(time_ms);
    MOTOR_STOP();
}

// 后退指定距离（单位：米）
void Motor_Backward_Distance(float distance_m)
{
    if(distance_m <= 0) return;
    uint32_t time_ms = (uint32_t)(distance_m * K_FORWARD_TIME);
    MOTOR_BACK(MOTOR_SPEED);
    HAL_Delay(time_ms);
    MOTOR_STOP();
}

// 左转指定角度（
void Motor_Turn_Left(int16_t angle_deg)
{
    if(angle_deg == 0) return;
    uint32_t time_ms = (uint32_t)(angle_deg * K_TURN_TIME);
    MOTOR_SPINL(MOTOR_SPEED); // 左转
    HAL_Delay(time_ms);
    MOTOR_STOP();
}

// 右转指定角度（
void Motor_Turn_Right(int16_t angle_deg)
{
    if(angle_deg == 0) return;
    uint32_t time_ms = (uint32_t)(angle_deg * K_TURN_TIME);
    MOTOR_SPINR(MOTOR_SPEED); // 右转
    HAL_Delay(time_ms);
    MOTOR_STOP();
}