#include "user_system.h"
#include "bsp_adc.h"
tSysDataDef tSysData;

typedef struct
{
    const char *sign;
    uint32_t integer;
    uint32_t fraction;
} tCurrentDecimal3;

static int32_t UserSystem_CurrentToMilliAmp(float current)
{
    if (current >= 0.0f)
    {
        return (int32_t)((current * 1000.0f) + 0.5f);
    }

    return (int32_t)((current * 1000.0f) - 0.5f);
}

static tCurrentDecimal3 UserSystem_CurrentToDecimal3(float current)
{
    int32_t milli_amp = UserSystem_CurrentToMilliAmp(current);
    uint32_t abs_milli_amp;
    tCurrentDecimal3 decimal;

    if (milli_amp < 0)
    {
        abs_milli_amp = (uint32_t)(-milli_amp);
        decimal.sign = "-";
    }
    else
    {
        abs_milli_amp = (uint32_t)milli_amp;
        decimal.sign = "";
    }

    decimal.integer = abs_milli_amp / 1000u;
    decimal.fraction = abs_milli_amp % 1000u;

    return decimal;
}


void UserSystem_Init(void)
{
    tSysData.enuState = E_SYS_STATE_INIT;
    tSysData.u32PowerOnTimes = 0u;
    tSysData.u32OpenTimes = 0u;
}


uint16_t PtTaskSystem(void)
{
    PT_BEGIN()
    {
        UserSystem_Init();
    }

    while (1)
    {
        tCurrentDecimal3 ia;
        tCurrentDecimal3 ib;
        tCurrentDecimal3 ic;
        tCurrentDecimal3 ibus;

        PT_WAIT_UNTIL(SYSTEM_TIME_MS / OS_TICK_MS);

        ia = UserSystem_CurrentToDecimal3(BspAdc_GetIa());
        ib = UserSystem_CurrentToDecimal3(BspAdc_GetIb());
        ic = UserSystem_CurrentToDecimal3(BspAdc_GetIc());
        ibus = UserSystem_CurrentToDecimal3(BspAdc_GetIbus());

        SEGGER_RTT_printf(0, "%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u\r\n",
                          ia.sign, (unsigned)ia.integer, (unsigned)ia.fraction,
                          ib.sign, (unsigned)ib.integer, (unsigned)ib.fraction,
                          ic.sign, (unsigned)ic.integer, (unsigned)ic.fraction,
                          ibus.sign, (unsigned)ibus.integer, (unsigned)ibus.fraction);
    }

    PT_END();
}
