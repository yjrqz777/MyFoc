/**
 * @file    user_display.c
 * @brief   用户显示任务实现
 *******************************************************************************
 * @note    利用 Protothread 协程实现周期性 LCD 显示刷新。
 *          LCD 显示内容包括：
 *          - Hall 传感器状态
 *          - 三相电流 + 母线电流 ADC 原始值
 *          - 4 个按键的按下状态
 *          - 4 个按键的上次事件类型
 *******************************************************************************
 */

#include "user_display.h"
#include "bsp_adc.h"
#include "bsp_hall.h"
#include "bsp_lcd.h"
#include "user_button.h"
#include "user_foc.h"
#include "SEGGER_RTT.h"
#include "Task.h"
#include "main.h"

/** @brief 显示更新周期（毫秒） */
#define USER_DISPLAY_PERIOD_MS 100u

/**
 * @brief  初始化显示模块
 * @note   初始化 LCD 显示屏
 */
void UserDisplay_Init(void)
{
    BspLcd_Init();
}

/**
 * @brief  每 100ms 更新一次 LCD 显示
 * @note   显示布局：
 *   - (0,24)   Hall 状态码
 *   - (0,48~120) ADC 注入通道 0~3 原始值（Ia, Ib, Ic, Ibus）
 *   - (96,24)  按键按下位掩码
 *   - (96,48~120) 按键 1~4 按下状态
 *   - (144,48~120) 按键 1~4 上次事件类型
 */
void UserDisplay_Update100ms(void)
{
    uint8_t hall_state = BspHall_GetState();
    DQCurrent_t dq = FOC_GetDQCurrent();
    (void)dq;

    /* 左列：Hall + ADC */
    BspLcd_ShowUInt(0, 24, hall_state, 1);
    BspLcd_ShowUInt(0, 48, BspAdc_GetInjectedRaw(0), 4);
    BspLcd_ShowUInt(0, 72, BspAdc_GetInjectedRaw(1), 4);
    BspLcd_ShowUInt(0, 96, BspAdc_GetInjectedRaw(2), 4);
    BspLcd_ShowUInt(0, 120, BspAdc_GetInjectedRaw(3), 4);

    /* 中列：按键状态 */
    BspLcd_ShowUInt(96, 24, UserButton_GetPressedMask(), 2);
    BspLcd_ShowUInt(96, 48, UserButton_GetPressed(1), 1);
    BspLcd_ShowUInt(96, 72, UserButton_GetPressed(2), 1);
    BspLcd_ShowUInt(96, 96, UserButton_GetPressed(3), 1);
    BspLcd_ShowUInt(96, 120, UserButton_GetPressed(4), 1);

    /* 右列：按键事件 */
    BspLcd_ShowUInt(144, 48, UserButton_GetLastEvent(1), 1);
    BspLcd_ShowUInt(144, 72, UserButton_GetLastEvent(2), 1);
    BspLcd_ShowUInt(144, 96, UserButton_GetLastEvent(3), 1);
    BspLcd_ShowUInt(144, 120, UserButton_GetLastEvent(4), 1);
}

/**
 * @brief  Protothread 显示协程任务
 * @return PT 状态码
 * @note   首次进入时执行初始化，
 *         之后循环等待 100ms 后刷新显示
 */
int PT_TASK_Display(void)
{
    PT_BEGIN()
    {
        UserDisplay_Init();
    }

    while (1)
    {
        PT_WAIT_UNTIL(100 / TIME_ms);
        UserDisplay_Update100ms();
    }

    PT_END();
}
