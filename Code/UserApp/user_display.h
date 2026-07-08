/**
 * @file    user_display.h
 * @brief   用户显示任务头文件 — Protothread 协程驱动 LCD 刷新
 *******************************************************************************
 * @note    每 100ms 刷新一次 LCD，显示 Hall 状态、ADC 值、按键信息等
 *******************************************************************************
 */

#ifndef __USER_DISPLAY_H__
#define __USER_DISPLAY_H__

#include "user_global.h"
#include "user_system.h"
#define USER_DISPLAY_PERIOD_MS 10u  /**< 显示刷新周期（毫秒） */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tDisplayFunDataDef
{
    enuSysState enuState;
    void (*fun)(void);
} tDisplayFunDataDef;

typedef struct tDisDataDef
{
    uint8_t u8color[3];
    uint16_t u16RGB;
} tDisDataDef;

extern tDisDataDef tDisData;

void UserDisplay_Init(void);
void UserDisplay_Update100ms(void);
// void UserDisplay_Poll(void);
uint16_t PtTaskDisplay(void);

#ifdef __cplusplus
}
#endif

#endif /* __USER_DISPLAY_H__ */
