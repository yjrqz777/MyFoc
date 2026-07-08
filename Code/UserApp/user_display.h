/**
 * @file    user_display.h
 * @brief   用户显示任务头文件 — Protothread 协程驱动 LCD 刷新
 *******************************************************************************
 * @note    每 100ms 刷新一次 LCD，显示 Hall 状态、ADC 值、按键信息等
 *******************************************************************************
 */

#ifndef __USER_DISPLAY_H__
#define __USER_DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

void UserDisplay_Init(void);
void UserDisplay_Update100ms(void);
void UserDisplay_Poll(void);
int PT_TASK_Display(void);

#ifdef __cplusplus
}
#endif

#endif /* __USER_DISPLAY_H__ */
