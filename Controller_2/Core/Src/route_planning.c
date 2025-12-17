#include "route_planning.h"
#include "lcd.h"
#include "screen.h"
#include <math.h>

#define CANVAS_SIZE   200
#define CANVAS_X0     20
#define CANVAS_Y0     60

extern UART_HandleTypeDef huart2;
static RoutePoint route_points[ROUTE_MAX_POINTS];
static uint16_t route_count = 0;

// 拟合后的关键点数组和数量
static RoutePoint fit_points[ROUTE_MAX_POINTS];
static uint16_t fit_count = 0;

// 坐标转换：像素->物理米
static RoutePoint pixel_to_meter(uint16_t px, uint16_t py)
{
    RoutePoint pt;
    pt.x = ((float)(px - CANVAS_X0) / CANVAS_SIZE) * AREA_SIZE_M;
    // 画布左下为(0,0)，LCD y轴向下，物理y轴向上
    pt.y = ((float)(CANVAS_Y0 + CANVAS_SIZE - py) / CANVAS_SIZE) * AREA_SIZE_M;
    return pt;
}

// 坐标转换：物理米->像素
static void meter_to_pixel(RoutePoint pt, uint16_t* px, uint16_t* py)
{
    *px = (uint16_t)(pt.x / AREA_SIZE_M * CANVAS_SIZE) + CANVAS_X0;
    *py = (uint16_t)(CANVAS_Y0 + CANVAS_SIZE - (pt.y / AREA_SIZE_M * CANVAS_SIZE));
}

void RoutePlanning_Init(void)
{
    route_count = 0;
    fit_count = 0;
}

void RoutePlanning_AddPoint(uint16_t px, uint16_t py)
{
    if(route_count < ROUTE_MAX_POINTS) {
        RoutePoint pt = pixel_to_meter(px, py);
        // 限制在2000mm*2000mm区域
        if(pt.x >= 0 && pt.x <= AREA_SIZE_M && pt.y >= 0 && pt.y <= AREA_SIZE_M) {
            // 只在与上一个点距离超过阈值时才记录
            if(route_count == 0) {
                route_points[route_count++] = pt;
            } else {
                RoutePoint last = route_points[route_count - 1];
                float dx = pt.x - last.x;
                float dy = pt.y - last.y;
                float dist = sqrtf(dx*dx + dy*dy);
                if(dist >= threshold) {
                    route_points[route_count++] = pt;
                }
            }
        }
    }
}

// 计算两点间方向角（弧度）
static float angle_rad(RoutePoint a, RoutePoint b)
{
    return atan2f(b.y - a.y, b.x - a.x);
}

// 弧度转角度
static float rad2deg(float r){ return r * 57.2957795f; }

// 对route_points进行折线拟合，结果存入fit_points，返回关键点数量
uint16_t RoutePlanning_FitByAngle(void)
{
    fit_count = 0;
    if(route_count < 2) return 0;

    fit_points[fit_count++] = route_points[0];
    float last_ang = angle_rad(route_points[0], route_points[1]);

    for(uint16_t i=2; i<route_count; i++)
    {
        float ang = angle_rad(route_points[i-1], route_points[i]);
        float d = rad2deg(ang - last_ang);
        while(d > 180) d -= 360;
        while(d < -180) d += 360;

        if(fabsf(d) >= TURN_THRESH_DEG)
        {
            if(fit_count < ROUTE_MAX_POINTS) fit_points[fit_count++] = route_points[i-1];
            last_ang = ang;
        }
    }
    if(fit_count < ROUTE_MAX_POINTS) fit_points[fit_count++] = route_points[route_count-1];
    return fit_count;
}

// 可选：获取拟合后的关键点
RoutePoint RoutePlanning_GetFitPoint(uint16_t idx)
{
    if(idx < fit_count) return fit_points[idx];
    RoutePoint zero = {0,0};
    return zero;
}


void RoutePlanning_Clear(void)
{
    route_count = 0;
    fit_count = 0;
}

uint16_t RoutePlanning_GetCount(void)
{
    return fit_count;
}

RoutePoint RoutePlanning_GetPoint(uint16_t idx)
{
    if(idx < route_count) return route_points[idx];
    RoutePoint zero = {0,0};
    return zero;
}

// 路径执行（直线+转弯指令，伪代码）
//通过串口2发送 '0' 控制前进，‘1'，后退，2' 控制左转，'3' 控制右转   通信协议
void RoutePlanning_Execute(void)
{
    if(fit_count < 2) return;
    float curr_angle = 90.0f; // 初始车头朝y轴正方向
    char txbuf[32];
    sprintf(txbuf, "$100,%d#", 0);
    HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 100);
    HAL_Delay(100);

    for(uint16_t i=1; i<fit_count; i++) {
        RoutePoint p0 = fit_points[i-1];
        RoutePoint p1 = fit_points[i];
        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        float dist = sqrtf(dx*dx + dy*dy); // 单位：米
        float target_angle = atan2f(dy, dx) * 180.0f / 3.1415926f;
        float turn_angle = target_angle - curr_angle;
        // 角度归一化到[-180,180]
        while(turn_angle > 180) turn_angle -= 360;
        while(turn_angle < -180) turn_angle += 360;

        // 发送转弯指令
        if(fabsf(turn_angle) > 1.0f) {
            int angle_int = (int)(fabsf(turn_angle) + 0.5f); // 四舍五入
            int type = (turn_angle > 0) ? 2 : 3; // 2=左转, 3=右转
            sprintf(txbuf, "$%d,%d#", type, angle_int);
//            Display_ShowMessage(txbuf); // 显示指令
            HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 100);
            HAL_Delay(100); // 可适当延时
        }

        // 发送前进指令
        if(dist > 0.01f) {
            int dist_mm = (int)(dist + 0.5f); // 米转毫米，四舍五入
            sprintf(txbuf, "$0,%d#", dist_mm);
            HAL_UART_Transmit(&huart2, (uint8_t*)txbuf, strlen(txbuf), 100);
            // Display_ShowMessage(txbuf); // 显示指令
            HAL_Delay(100); // 可适当延时
        }

        curr_angle = target_angle;
    }
}
