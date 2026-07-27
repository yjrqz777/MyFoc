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

tDisplayDataDef tDisplayData;

#define DISPLAY_WS2812_LEVEL (25u)

static uint8_t u8PowerOnInitDone = 0u;

static void UsrDisplaySetWs2812Color(uint8_t u8Red, uint8_t u8Green, uint8_t u8Blue)
{
    /* Refresh periodically so the short WS2812 frame remains observable. */
    (void)BspWs2812WriteColor(u8Red, u8Green, u8Blue);
}

/**
 * @brief  初始化显示模块
 * @note   初始化 LCD 显示屏
 */
void UsrDisplayInit(void)
{
    BspLcdInit();
    BspWs2812Init();
}

/**
 * @brief  系统初始化状态下的显示处理
 * @note   设置 WS2812 为黄色（RGB 全亮），
 *         在 LCD 上显示 "FOC" 标题（仅首次执行）
 */
static void UsrDisplayInitState(void)
{
    static uint8_t u8InitDone = 0u;

    tDisplayData.u8Color[0] = DISPLAY_WS2812_LEVEL;
    tDisplayData.u8Color[1] = DISPLAY_WS2812_LEVEL;
    tDisplayData.u8Color[2] = 0u;

    if (u8InitDone == 0u)
    {
        u8InitDone = 1u;
        LCD_ShowChineseTEST(0, 0, "FOC", BLACK, WHITE, 80,135, 0);
    }
}

/**
 * @brief  上电状态下的显示处理
 * @note   设置 WS2812 为红色，
 *         在 LCD 上显示作者信息（仅首次执行）
 */
static void UsrDisplayPowerOnState(void)
{
    tDisplayData.u8Color[0] = DISPLAY_WS2812_LEVEL;
    tDisplayData.u8Color[1] = 0u;
    tDisplayData.u8Color[2] = 0u;

    if (u8PowerOnInitDone == 0u)
    {
        u8PowerOnInitDone = 1u;
        LCD_ShowChineseTEST(0, 0, "FOC", BLACK, WHITE, 80,135, 0);
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
static void UsrDisplayRunningState(void)
{
    static uint8_t u8TimeCount = 0u;
    static uint16_t u16TimeCount = 0u;
    uint8_t HallState;
    tDqCurrentDef DqCurrent;
    uint16_t IqReferenceMilliAmp;
    uint16_t IqTargetMilliAmp;

    u8PowerOnInitDone = 0u;
    HallState = BspHallGetState();
    DqCurrent = UsrFocGetDqCurrent();
    (void)HallState;
    (void)DqCurrent;

    tDisplayData.u8Color[0] = 0u;
    tDisplayData.u8Color[1] = DISPLAY_WS2812_LEVEL;
    tDisplayData.u8Color[2] = 0u;
    u8TimeCount++;
    if (u8TimeCount < 100u / USR_DISPLAY_TASK_INTERVAL_MS)
    {
        return;
    }
    u8TimeCount = 0u;

    /* 左列：电位器 ADC + Iq 参考值 */
    BspLcdShowUInt(0u, 48u, BspAdc2GetRaw(E_BSP_ADC2_POT), 4u);
    IqReferenceMilliAmp = (uint16_t)(UsrMotorGetIqRef() * 1000.0f);
    IqTargetMilliAmp = (uint16_t)(UsrMotorGetIqRefTarget() * 1000.0f);
    BspLcdShowUInt(0u, 72u, IqReferenceMilliAmp, 4u);
    BspLcdShowUInt(0u, 96u, IqTargetMilliAmp, 4u);

    /* 中列：按键状态 + 运行计数器 */
    BspLcdShowUInt(96u, 0u, UsrButtonGetPressed(2u), 1u);
    BspLcdShowUInt(120u, 0u, u16TimeCount++, 6u);
}

/**
 * @brief  关闭状态下的显示处理
 * @note   当前为空操作，预留用于关闭显示或进入低功耗模式
 */
static void UsrDisplayOffState(void)
{
}

/**
 * @brief  Protothread 显示协程任务
 * @return PT 状态码
 * @note   首次进入时执行初始化，之后循环等待显示周期后刷新显示
 */
uint16_t UsrDisplayTask(void)
{
    int8_t i = 0;
    tDisplayFunctionDataDef DisplayFunction[E_SYS_STATE_MAX] = {
        {E_SYS_STATE_INIT, UsrDisplayInitState},
        {E_SYS_STATE_POWER_ON, UsrDisplayPowerOnState},
        {E_SYS_STATE_RUNNING, UsrDisplayRunningState},
        {E_SYS_STATE_OFF, UsrDisplayOffState},
    };

    PT_BEGIN()
    {
        UsrDisplayInit();
    }

    while (1)
    {
        PT_WAIT_UNTIL(USR_DISPLAY_TASK_INTERVAL_MS / OS_TICK_MS);

        // 显示数据复位
        memset(&tDisplayData, 0, sizeof(tDisplayData));

        for (i = (sizeof(DisplayFunction) / sizeof(DisplayFunction[0])) - 1; i >= 0; i--)
        {
            if (tSysData.eState == DisplayFunction[i].eState)
            {
                DisplayFunction[i].pfFunction();
                break;
            }
        }
        // SEGGER_RTT_printf(0, "%d,%d,%d\r\n", tSysData.eState,tSysData.u32OpenTimes, tSysData.u32PowerOnTimes);
        UsrDisplaySetWs2812Color(tDisplayData.u8Color[0],
                                 tDisplayData.u8Color[1],
                                 tDisplayData.u8Color[2]);
    }
    PT_END();
}
