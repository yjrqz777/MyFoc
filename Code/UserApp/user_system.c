#include "user_system.h"
#include "bsp_adc.h"
#include "user_foc.h"
#include "user_motor.h"
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
    SEGGER_RTT_WriteString(0, "Ia,Ib,Ic,Id,Iq,s_iq_ref,iq_ref_target\r\n");
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
        tCurrentDecimal3 id;
        tCurrentDecimal3 iq;
        tCurrentDecimal3 iq_ref;
        tCurrentDecimal3 iq_target;
        DQCurrent_t dq;

        PT_WAIT_UNTIL(SYSTEM_TIME_MS / OS_TICK_MS);

        (void)BspAdc2_UpdateAll();

        ia = UserSystem_CurrentToDecimal3(BspAdc_GetIa());
        ib = UserSystem_CurrentToDecimal3(BspAdc_GetIb());
        ic = UserSystem_CurrentToDecimal3(BspAdc_GetIc());
        dq = FOC_GetDQCurrent();
        id = UserSystem_CurrentToDecimal3(dq.d);
        iq = UserSystem_CurrentToDecimal3(dq.q);
        iq_ref = UserSystem_CurrentToDecimal3(UserMotor_GetIqRef());
        iq_target = UserSystem_CurrentToDecimal3(UserMotor_GetIqRefTarget());

        SEGGER_RTT_printf(0, "%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u\r\n",
                          ia.sign, (unsigned)ia.integer, (unsigned)ia.fraction,
                          ib.sign, (unsigned)ib.integer, (unsigned)ib.fraction,
                          ic.sign, (unsigned)ic.integer, (unsigned)ic.fraction,
                          id.sign, (unsigned)id.integer, (unsigned)id.fraction,
                          iq.sign, (unsigned)iq.integer, (unsigned)iq.fraction,
                          iq_ref.sign, (unsigned)iq_ref.integer, (unsigned)iq_ref.fraction,
                          iq_target.sign, (unsigned)iq_target.integer, (unsigned)iq_target.fraction);
    }

    PT_END();
}
