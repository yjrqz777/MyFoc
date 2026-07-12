/**
 * @file    user_button_fun.c
 * @brief   按键功能回调实现
 *******************************************************************************
 * @note    实现按键事件触发的上层功能回调，
 *          当前用于系统状态切换（POWER_ON ↔ RUNNING）。
 *          通过 UserStatusSwitch() 回调与 user_button 的事件绑定关联。
 *******************************************************************************
 */

#include "user_button.h"
#include "bsp_button.h"
#include "user_system.h"

/**
 * @brief  按键状态切换回调函数
 * @param[in] btn  触发事件的按键结构体指针
 * @note   绑定到按键1（KEY1）的单击事件（BTN_SINGLE_CLICK）。
 *         在 POWER_ON 和 RUNNING 状态之间切换。
 *         其他状态下不作响应。
 * @see    buttons_init() 中的 button_attach 绑定
 */
void UserStatusSwitch(Button *btn)
{
    (void)btn;

    if (tSysData.enuState == E_SYS_STATE_POWER_ON)
    {
        tSysData.enuState = E_SYS_STATE_RUNNING;
        return;
    }

    if (tSysData.enuState == E_SYS_STATE_RUNNING)
    {
        tSysData.enuState = E_SYS_STATE_POWER_ON;
        return;
    }
}