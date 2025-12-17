#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim4;


typedef enum {
    CMD_TURN_L = 0,
    CMD_TURN_R = 1,
    CMD_FORWARD = 2,
    CMD_END = 3,
    CMD_ABORT = 4
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    uint16_t value;   // 角度(度) 或 距离(mm)
} cmd_t;

// 全局参数：默认占空比与时间常数（由 ir_follow 等模块复用）
#ifndef MOTOR_SPEED
#define MOTOR_SPEED      60      // 电机PWM占空比（0~100）
#endif

#ifndef K_FORWARD_TIME
#define K_FORWARD_TIME   3.5f    // 每毫米所需时间(ms/mm)，需标定
#endif

#ifndef K_TURN_TIME
#define K_TURN_TIME      7.5f    // 每度所需时间(ms/deg)，需标定
#endif


#define MotorCalibrationCoeff 0.925f // At 7.8V+
// #define MotorCalibrationCoeff 0.9000f  // Below 7.8V+

#define __SetMotorSpeed(__CHANNEL__, __PORT__, __PIN__, __speed__)                                                     \
    do {                                                                                                               \
        __HAL_TIM_SetCompare(&htim4, (__CHANNEL__),                                                                    \
                             (int32_t)(72 * ((__speed__) < 0 ? -(__speed__) : 100 - (__speed__))));                    \
        HAL_GPIO_WritePin((__PORT__), (__PIN__), ((__speed__) < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET));                   \
    } while (0)
#define __GetMotorSpeed(__CHANNEL__) __HAL_TIM_GetCompare(&htim4, (__CHANNEL__))

#define SetMotorSpeedLF(speed)                                                                                         \
    __SetMotorSpeed(TIM_CHANNEL_1, LEFT1_MOTOR_FORWARD_GPIO_Port, LEFT1_MOTOR_FORWARD_Pin, ((speed) * MotorCalibrationCoeff))

#define SetMotorSpeedLB(speed)                                                                                         \
    __SetMotorSpeed(TIM_CHANNEL_2, LEFT2_MOTOR_FORWARD_GPIO_Port, LEFT2_MOTOR_FORWARD_Pin, ((speed) * MotorCalibrationCoeff))

#define SetMotorSpeedRF(speed)                                                                                         \
    __SetMotorSpeed(TIM_CHANNEL_3, RIGHT1_MOTOR_FORWARD_GPIO_Port, RIGHT1_MOTOR_FORWARD_Pin, speed)

#define SetMotorSpeedRB(speed)                                                                                         \
    __SetMotorSpeed(TIM_CHANNEL_4, RIGHT2_MOTOR_FORWARD_GPIO_Port, RIGHT2_MOTOR_FORWARD_Pin, speed)

#define GetMotorSpeedLF() __GetMotorSpeed(TIM_CHANNEL_1)

#define GetMotorSpeedLB() __GetMotorSpeed(TIM_CHANNEL_2)

#define GetMotorSpeedRF() __GetMotorSpeed(TIM_CHANNEL_3)

#define GetMotorSpeedRB() __GetMotorSpeed(TIM_CHANNEL_4)

#define MOTOR_SPINL(speed)                                                                                             \
    do {                                                                                                               \
        SetMotorSpeedLF(-(speed));                                                                                     \
        SetMotorSpeedLB(-(speed));                                                                                     \
        SetMotorSpeedRF(speed);                                                                                        \
        SetMotorSpeedRB(speed);                                                                                        \
    } while (0)

#define MOTOR_SPINR(speed) MOTOR_SPINL(-(speed))

#define MOTOR_TURNL(speed)                                                                                             \
    do {                                                                                                               \
        SetMotorSpeedLF(0);                                                                                            \
        SetMotorSpeedLB(0);                                                                                            \
        SetMotorSpeedRF(speed);                                                                                        \
        SetMotorSpeedRB(speed);                                                                                        \
    } while (0)

#define MOTOR_TURNR(speed)                                                                                             \
    do {                                                                                                               \
        SetMotorSpeedLF(speed);                                                                                        \
        SetMotorSpeedLB(speed);                                                                                        \
        SetMotorSpeedRF(0);                                                                                            \
        SetMotorSpeedRB(0);                                                                                            \
    } while (0)

#define MOTOR_FORWARD(speed)                                                                                           \
    do {                                                                                                               \
        SetMotorSpeedLF(speed);                                                                                        \
        SetMotorSpeedLB(speed);                                                                                        \
        SetMotorSpeedRF(speed);                                                                                        \
        SetMotorSpeedRB(speed);                                                                                        \
    } while (0)

#define MOTOR_BACK(speed) MOTOR_FORWARD(-(speed))

#define MOTOR_LEFT(speed)                                                                                              \
    do {                                                                                                               \
        SetMotorSpeedLF(-(speed));                                                                                            \
        SetMotorSpeedLB(-(speed));                                                                                            \
        SetMotorSpeedRF(speed);                                                                                        \
        SetMotorSpeedRB(speed);                                                                                      \
    } while (0)

#define MOTOR_RIGHT(speed)                                                                                             \
    do {                                                                                                               \
        SetMotorSpeedLF(speed);                                                                                            \
        SetMotorSpeedLB(speed);                                                                                            \
        SetMotorSpeedRF(-(speed));                                                                                        \
        SetMotorSpeedRB(-(speed));                                                                                       \
    } while (0)

#define MOTOR_STOP()                                                                                                   \
    do {                                                                                                               \
        SetMotorSpeedLF(0);                                                                                            \
        SetMotorSpeedLB(0);                                                                                            \
        SetMotorSpeedRF(0);                                                                                            \
        SetMotorSpeedRB(0);                                                                                            \
    } while (0)

#define MOTOR_FORWARD_L(speed_forward, speed_L)                                                                        \
    do {                                                                                                               \
        SetMotorSpeedLF((speed_forward) - (speed_L));                                                                  \
        SetMotorSpeedLB((speed_forward) - (speed_L));                                                                  \
        SetMotorSpeedRF(speed_forward);                                                                                \
        SetMotorSpeedRB(speed_forward);                                                                                \
    } while (0)

#define MOTOR_FORWARD_R(speed_forward, speed_R)                                                                        \
    do {                                                                                                               \
        SetMotorSpeedLF(speed_forward);                                                                                \
        SetMotorSpeedLB(speed_forward);                                                                                \
        SetMotorSpeedRF((speed_forward) - (speed_R));                                                                  \
        SetMotorSpeedRB((speed_forward) - (speed_R));                                                                  \
    } while (0)

#define MOTOR_BACK_L(speed_back, speed_L)                                                                              \
    do {                                                                                                               \
        SetMotorSpeedLF(-(speed_back) + (speed_L));                                                                    \
        SetMotorSpeedLB(-(speed_back) + (speed_L));                                                                    \
        SetMotorSpeedRF(-(speed_back));                                                                                \
        SetMotorSpeedRB(-(speed_back));                                                                                \
    } while (0)

#define MOTOR_BACK_R(speed_back, speed_R)                                                                              \
    do {                                                                                                               \
        SetMotorSpeedLF(-(speed_back));                                                                                \
        SetMotorSpeedLB(-(speed_back));                                                                                \
        SetMotorSpeedRF(-(speed_back) + (speed_R));                                                                    \
        SetMotorSpeedRB(-(speed_back) + (speed_R));                                                                    \
    } while (0)

#define MOTOR_SET_SPD(speed_L, speed_R)                                                                                \
    do {                                                                                                               \
        SetMotorSpeedLF((speed_L));                                                                                    \
        SetMotorSpeedLB((speed_L));                                                                                    \
        SetMotorSpeedRF((speed_R));                                                                                    \
        SetMotorSpeedRB((speed_R));                                                                                    \
    } while (0)

#ifdef __cplusplus
extern "C" {
#endif

// 函数封装，供其他模块调用，避免在其编译单元展开 GPIO 宏
void Motor_SetSpeedLR(int speed_L, int speed_R);
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif