/**
 * @file    user_time.c
 * @brief   系统时间管理任务实现
 *******************************************************************************
 * @note    提供系统计时功能，包括上电计时和开机计时。
 *          上电 100ms 后自动将系统状态从 INIT 切换至 POWER_ON。
 *          通过 Protothread 协程周期性更新时间计数。
 *******************************************************************************
 */

#include "user_time.h"
#include "user_system.h"

/**
 * @brief  初始化系统计时数据
 * @note   清零上电计时和开机计时，在系统启动时调用一次
 */
void UserTime_Init(void)
{
    tSysData.u32PowerOnTimes = 0u;
    tSysData.u32OpenTimes = 0u;
}

/**
 * @brief  更新系统时间计数和状态机
 * @note   按 TIME_TIME_MS 周期调用，执行以下逻辑：
 *         - 累计上电时间（u32PowerOnTimes）
 *         - 上电 100ms 后将系统状态从 INIT 切换至 POWER_ON
 *         - 当状态达到 RUNNING 后累计开机时间（u32OpenTimes）
 * @see    PtTaskTime
 */
void UserSysTimeUpdate(void)
{
    tSysData.u32PowerOnTimes += TIME_TIME_MS;

    if (tSysData.enuState == E_SYS_STATE_INIT && tSysData.u32PowerOnTimes > 100u)
    {
        tSysData.enuState = E_SYS_STATE_POWER_ON;
    }

    if (tSysData.enuState < E_SYS_STATE_RUNNING)
    {
        return;
    }

    tSysData.u32OpenTimes += TIME_TIME_MS;
}

/**
 * @brief  Protothread 系统时间协程任务
 * @return PT 状态码
 * @note   首次进入时初始化计时，之后按 TIME_TIME_MS 周期
 *         调 UserSysTimeUpdate() 更新系统计时
 */
uint16_t PtTaskTime(void)
{
    PT_BEGIN()
    {
        UserTime_Init();
    }

    while (1)
    {
        PT_WAIT_UNTIL(TIME_TIME_MS / OS_TICK_MS);
        UserSysTimeUpdate();
    }

    PT_END();
}

