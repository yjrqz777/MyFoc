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
#include "bsp_ws2812.h"
#include "user_button.h"
#include "user_foc.h"
#include "user_motor.h"
#include "st7789v/st7789v.h"

tDisDataDef tDisData;

#define DISPLAY_WS2812_LEVEL 255u

static void UserDisplay_SetWs2812Color(uint8_t red, uint8_t green, uint8_t blue)
{
    static uint8_t last_valid = 0u;
    static uint8_t last_red = 0u;
    static uint8_t last_green = 0u;
    static uint8_t last_blue = 0u;

    if ((last_valid != 0u) &&
        (last_red == red) &&
        (last_green == green) &&
        (last_blue == blue))
    {
        return;
    }

    if (BspWs2812_WriteColor(red, green, blue) == HAL_OK)
    {
        last_valid = 1u;
        last_red = red;
        last_green = green;
        last_blue = blue;
    }
}

/** @brief 显示更新周期（毫秒） */

/**
 * @brief  初始化显示模块
 * @note   初始化 LCD 显示屏
 */
void UserDisplay_Init(void)
{
    BspLcd_Init();
    // BspWs2812_Init();
}


/**
 * @brief  系统初始化状态下的显示处理
 * @note   设置 WS2812 为黄色（RGB 全亮），
 *         在 LCD 上显示 "FOC" 标题（仅首次执行）
 */
void DisplayInit(void)
{
    static uint8_t init_done = 0u;

    tDisData.u8color[0] = DISPLAY_WS2812_LEVEL;
    tDisData.u8color[1] = DISPLAY_WS2812_LEVEL;
    tDisData.u8color[2] = 0;

    if (init_done == 0)
    {
        init_done = 1;
        LCD_ShowChineseTEST(0, 0, "FOC", BLACK, WHITE, 80,135, 0);
    }
}

/**
 * @brief  上电状态下的显示处理
 * @note   设置 WS2812 为红色，
 *         在 LCD 上显示作者信息（仅首次执行）
 */
void DisplayPowerOn(void)
{
    static uint8_t init_done = 0u;

    tDisData.u8color[0] = DISPLAY_WS2812_LEVEL;
    tDisData.u8color[1] = 0;
    tDisData.u8color[2] = 0;

    if (init_done == 0)
    {
        init_done = 1;
        LCD_ShowString(0, 0, "YJRQZ777", BLACK, WHITE, 16, 0);
    }

}

/**
 * @brief  运行状态下的显示处理
 * @note   设置 WS2812 为绿色，按周期刷新 LCD：
 *         - 左列：电位器 ADC 值、Iq 参考值和目标值
 *         - 中列：按键 2 按下状态、运行计数器
 *         实际刷新间隔 100ms（累积显示周期后输出）
 */
void DisplayRunning(void)
{
    static uint8_t u8timecount = 0u;
    static uint16_t u8timecount2 = 0u;
    uint8_t hall_state = BspHall_GetState();
    DQCurrent_t dq = FOC_GetDQCurrent();
    uint16_t iq_ref_ma;
    uint16_t iq_target_ma;
    (void)dq;


    tDisData.u8color[0] = 0;
    tDisData.u8color[1] = DISPLAY_WS2812_LEVEL;
    tDisData.u8color[2] = 0;
    u8timecount++;
    if (u8timecount < 100/USER_DISPLAY_PERIOD_MS)
    {
        return;
    }
    u8timecount = 0;

    /* 左列：电位器 ADC + Iq 参考值 */
    BspLcd_ShowUInt(0, 48, BspAdc2_GetRaw(BSP_ADC2_POT), 4);
    iq_ref_ma = (uint16_t)(UserMotor_GetIqRef() * 1000.0f);
    iq_target_ma = (uint16_t)(UserMotor_GetIqRefTarget() * 1000.0f);
    BspLcd_ShowUInt(0, 72, iq_ref_ma, 4);
    BspLcd_ShowUInt(0, 96, iq_target_ma, 4);

    /* 中列：按键状态 + 运行计数器 */
    BspLcd_ShowUInt(96, 0, UserButton_GetPressed(2), 1);
    BspLcd_ShowUInt(120, 0, u8timecount2++, 6);
}

/**
 * @brief  关闭状态下的显示处理
 * @note   当前为空操作，预留用于关闭显示或进入低功耗模式
 */
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
        // UserDisplay_SetWs2812Color(tDisData.u8color[0], tDisData.u8color[1], tDisData.u8color[2]);

        // DisPlayDrive();

    }
    PT_END();
}
