/**
 * @file    user_button.h
 * @brief   用户按键管理头文件 — 4 按键事件管理
 *******************************************************************************
 * @note    封装 multi-button 库，支持单击/双击/长按/重复等事件
 *******************************************************************************
 */

#ifndef USER_BUTTON_H
#define USER_BUTTON_H

#include "user_global.h"

#define BUTTON_TIME_MS  5    // ms - timer interrupt interval

void buttons_init(void);
uint8_t UserButton_GetRawMask(void);
uint8_t UserButton_GetPressed(uint8_t button_id);
uint8_t UserButton_GetPressedMask(void);
uint8_t UserButton_GetLastEvent(uint8_t button_id);
uint16_t PtTaskButton(void);

#endif /* USER_BUTTON_H */
