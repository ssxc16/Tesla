#ifndef __ROUTE_PLANNING_H
#define __ROUTE_PLANNING_H

#include "stm32f1xx_hal.h"

#define CANVAS_SIZE   200
#define CANVAS_X0     20
#define CANVAS_Y0     60

#define ROUTE_MAX_POINTS 1000
#define TURN_THRESH_DEG 20.0f // 转弯阈值
#define threshold 50.0f // 500毫米
#define AREA_SIZE_M   2000.0f // 2000mm x 2000mm

#define K_FORWARD 1000  // 前进1米所需时间ms（需实测调整）
#define K_TURN    10    // 转动1度所需时间ms（需实测调整）

typedef struct {
    float x; // 单位：米
    float y; // 单位：米
} RoutePoint;

// 初始化路线规划
void RoutePlanning_Init(void);

// 添加一个画布触摸点（像素坐标），自动转换为物理坐标
void RoutePlanning_AddPoint(uint16_t px, uint16_t py);

// 清除所有路线点
void RoutePlanning_Clear(void);


uint16_t RoutePlanning_GetCount(void);

RoutePoint RoutePlanning_GetPoint(uint16_t idx);

uint16_t RoutePlanning_FitByAngle(void);
RoutePoint RoutePlanning_GetFitPoint(uint16_t idx);

// 将路线点转换为运动指令（直线+转弯），并下发给小车
void RoutePlanning_Execute(void);


#endif