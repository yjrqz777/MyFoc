#include "user_time.h"
#include "user_system.h"

void UserTime_Init(void)
{
    tSysData.u32PowerOnTimes = 0u;
    tSysData.u32OpenTimes = 0u;
}

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

