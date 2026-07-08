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
#include "user_system.h"
#include "bsp_adc.h"
#include "bsp_hall.h"
#include "bsp_lcd.h"
#include "user_button.h"
#include "user_foc.h"
#include "st7789v/st7789v.h"

tDisDataDef tDisData;

/** @brief 显示更新周期（毫秒） */

/**
 * @brief  初始化显示模块
 * @note   初始化 LCD 显示屏
 */
void UserDisplay_Init(void)
{
    BspLcd_Init();
}


void DisplayInit(void)
{
    static uint8_t init_done = 0u;
    if (init_done == 0)
    {
        init_done = 1;
        LCD_ShowChineseTEST(0, 0, "FOC", BLACK, WHITE, 80,135, 0);
    }
}
void DisplayPowerOn(void)
{
    static uint8_t init_done = 0u;
    if (init_done == 0)
    {
        init_done = 1;
        LCD_ShowString(0, 0, "YJRQZ777", BLACK, WHITE, 16, 0);
    }
    
}


void DisplayRunning(void)
{
    static uint8_t u8timecount = 0u;
    uint8_t hall_state = BspHall_GetState();
    DQCurrent_t dq = FOC_GetDQCurrent();
    (void)dq;

    u8timecount++;
    if (u8timecount < 100/USER_DISPLAY_PERIOD_MS)
    {
        return;
    }
    u8timecount = 0;


    /* 左列：Hall + ADC */
    // BspLcd_ShowUInt(0, 24, hall_state, 1);
    // BspLcd_ShowUInt(0, 48, BspAdc_GetInjectedRaw(0), 4);
    // BspLcd_ShowUInt(0, 72, BspAdc_GetInjectedRaw(1), 4);
    // BspLcd_ShowUInt(0, 96, BspAdc_GetInjectedRaw(2), 4);
    // BspLcd_ShowUInt(0, 120, BspAdc_GetInjectedRaw(3), 4);

    /* 中列：按键状态 */
    // BspLcd_ShowUInt(96, 24, UserButton_GetPressedMask(), 2);
    // BspLcd_ShowUInt(96, 48, UserButton_GetPressed(1), 1);
    // BspLcd_ShowUInt(96, 72, UserButton_GetPressed(2), 1);
    // BspLcd_ShowUInt(96, 96, UserButton_GetPressed(3), 1);
    // BspLcd_ShowUInt(96, 120, UserButton_GetPressed(4), 1);

    /* 右列：按键事件 */
    // BspLcd_ShowUInt(144, 48, UserButton_GetLastEvent(1), 1);
    // BspLcd_ShowUInt(144, 72, UserButton_GetLastEvent(2), 1);
    // BspLcd_ShowUInt(144, 96, UserButton_GetLastEvent(3), 1);
    // BspLcd_ShowUInt(144, 120, UserButton_GetLastEvent(4), 1);
}

void DisplayOff(void)
{
    // UserDisplay_Update100ms();
}

/**
 * @brief  Protothread 显示协程任务
 * @return PT 状态码
 * @note   首次进入时执行初始化，
 *         之后循环等待 100ms 后刷新显示
 */
uint16_t PtTaskDisplay(void)
{
    int8_t i = 0;
    tDisplayFunDataDef DisplayFun[E_SYS_STATE_MAX] = {
        {E_SYS_STATE_INIT, DisplayInit},
        {E_SYS_STATE_POWER_ON, DisplayPowerOn},
        {E_SYS_STATE_RUNNING, DisplayRunning},
        {E_SYS_STATE_OFF, DisplayOff},
    };

    PT_BEGIN()
    {
        UserDisplay_Init();
    }

    while (1)
    {
        PT_WAIT_UNTIL(USER_DISPLAY_PERIOD_MS / OS_TICK_MS);

        // 显示数据复位
        memset(&tDisData, 0, sizeof(tDisData));

        for (i = (sizeof(DisplayFun) / sizeof(DisplayFun[0])) - 1; i >= 0; i--)
        {
            if (tSysData.enuState == DisplayFun[i].enuState)
            {
                DisplayFun[i].fun();
                break;
            }
        }
        // SEGGER_RTT_printf(0, "%d,%d,%d\r\n", tSysData.enuState,tSysData.u32OpenTimes, tSysData.u32PowerOnTimes);
        

        // DisPlayDrive();

    }
    PT_END();
}
