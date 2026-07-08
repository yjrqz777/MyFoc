#ifndef __USER_SYSTEM_H__
#define __USER_SYSTEM_H__

#include "user_global.h"
#define SYSTEM_TIME_MS 5
typedef enum enuSysState
{
    E_SYS_STATE_INIT = 0,
    E_SYS_STATE_POWER_ON,
    E_SYS_STATE_RUNNING,
    E_SYS_STATE_OFF,
    E_SYS_STATE_MAX,
} enuSysState;

typedef struct tSysDataDef
{
    enuSysState enuState;
    uint32_t u32PowerOnTimes;  /**< 上电时间计数（单位：ms） */
    uint32_t u32OpenTimes;     /**< 开机时间计数（单位：ms） */
} tSysDataDef;

extern tSysDataDef tSysData;
uint16_t PtTaskSystem(void);
#endif /* __USER_SYSTEM_H__ */
