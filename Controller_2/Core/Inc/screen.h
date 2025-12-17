#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "stm32f1xx_hal.h"
#include <stdint.h>


void rtp_test(void);
// 显示连接状态
void Display_ConnectionStatus(uint8_t connected);

// 显示主菜单
void Display_ShowMainMenu(void);

// 显示子菜单
void Display_ShowSubMenu(const char* title);

// 显示提示信息
void Display_ShowMessage(const char* msg);

void RoutePlanning(void);

#endif