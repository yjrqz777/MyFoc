/**
 * @file    user_button_fun.c
 * @brief   按键功能回调与冻结实现兼容门面
 *******************************************************************************
 * @note    实现按键事件触发的上层功能回调，并为不可修改的
 *          user_button.c 导出真实兼容符号对应的规范接口。
 *******************************************************************************
 */

#include "user_button.h"
#include "bsp_button.h"
#include "user_system.h"

/**
 * @brief  按键状态切换回调函数
 * @param[in] ptButton  触发事件的按键结构体指针
 * @note   绑定到按键1（KEY1）的单击事件（BTN_SINGLE_CLICK）。
 *         在 POWER_ON 和 RUNNING 状态之间切换。
 *         其他状态下不作响应。
 */
void UsrButtonStatusSwitch(Button * ptButton)
{
    (void)ptButton;

    if (tSysData.eState == E_SYS_STATE_POWER_ON)
    {
        tSysData.eState = E_SYS_STATE_RUNNING;
        return;
    }

    if (tSysData.eState == E_SYS_STATE_RUNNING)
    {
        tSysData.eState = E_SYS_STATE_POWER_ON;
        return;
    }
}

/** @brief 冻结 user_button.c 回调所需的真实兼容符号 */
void UserStatusSwitch(Button * ptButton)
{
    UsrButtonStatusSwitch(ptButton);
}

void UsrButtonInit(void)
{
    buttons_init();
}

uint8_t UsrButtonGetRawMask(void)
{
    return UserButton_GetRawMask();
}

uint8_t UsrButtonGetPressed(uint8_t u8ButtonId)
{
    return UserButton_GetPressed(u8ButtonId);
}

uint8_t UsrButtonGetPressedMask(void)
{
    return UserButton_GetPressedMask();
}

uint8_t UsrButtonGetLastEvent(uint8_t u8ButtonId)
{
    return UserButton_GetLastEvent(u8ButtonId);
}

uint16_t UsrButtonTask(void)
{
    return PtTaskButton();
}
