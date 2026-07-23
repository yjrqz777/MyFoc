/**
 * @file    user_system.c
 * @brief   系统任务与状态管理实现
 *******************************************************************************
 * @note    提供系统初始化、状态机管理以及电流数据格式化输出的功能。
 *          通过 Protothread 协程周期性采集并输出电流数据至 RTT。
 *          系统状态包括 INIT -> POWER_ON -> RUNNING -> OFF 四个阶段。
 *******************************************************************************
 */

#include "user_system.h"
#include "bsp_adc.h"
#include "user_foc.h"
#include "user_motor.h"
tSysDataDef tSysData;

/** @brief 三位小数电流格式化结构体（用于 RTT CSV 输出） */
typedef struct
{
    const char *sign;      /**< 符号串，"" 或 "-" */
    uint32_t integer;      /**< 整数部分（A） */
    uint32_t fraction;     /**< 小数部分（毫安，千分位） */
} tCurrentDecimal3;

/**
 * @brief  将电流值（A）转换为毫安整数（四舍五入）
 * @param[in] current  电流值，单位 A
 * @return 电流的毫安等效值（正数向上取整，负数向下取整）
 * @note   用于 RTT 输出的格式化转换
 */
static int32_t UserSystem_CurrentToMilliAmp(float current)
{
    if (current >= 0.0f)
    {
        return (int32_t)((current * 1000.0f) + 0.5f);
    }

    return (int32_t)((current * 1000.0f) - 0.5f);
}

/**
 * @brief  将电流值（A）转换为带符号和小数部分的十进制结构体
 * @param[in] current  电流值，单位 A
 * @return tCurrentDecimal3 结构体，包含符号串、整数部分和小数部分（千分位）
 * @note   用于 RTT CSV 格式输出，格式如 "-1.234" 或 "0.567"
 */
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


/**
 * @brief  初始化系统状态和数据
 * @note   设置系统状态为 E_SYS_STATE_INIT，清零运行计数，
 *         并通过 RTT 输出电流 CSV 格式标题行，供 J-Scope 数据采集
 */
void UserSystem_Init(void)
{
    tSysData.enuState = E_SYS_STATE_INIT;
    tSysData.u32PowerOnTimes = 0u;
    tSysData.u32OpenTimes = 0u;
    SEGGER_RTT_WriteString(0, "Ia,Ib,Ic,Id,Iq,s_iq_ref,iq_ref_target,oc_fault,fault_ia,fault_ib,fault_ic,rawA,rawB,rawC\r\n");
}


/**
 * @brief  Protothread 系统协程任务 — 周期性采集并输出电流数据
 * @return PT 状态码
 * @note   首次进入时初始化系统，之后按 SYSTEM_TIME_MS 周期执行：
 *         - 轮询更新 ADC2 通道值（母线电压、电位器等）
 *         - 采集三相电流 (Ia/Ib/Ic) 和 dq 轴电流 (Id/Iq)
 *         - 通过 RTT 输出 CSV 格式数据流，供上位机或 J-Scope 采集
 */
uint16_t PtTaskSystem(void)
{
    static uint8_t offset_printed = 0u;

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
        tCurrentDecimal3 fault_ia;
        tCurrentDecimal3 fault_ib;
        tCurrentDecimal3 fault_ic;
        DQCurrent_t dq;
        uint16_t raw_a;
        uint16_t raw_b;
        uint16_t raw_c;

        PT_WAIT_UNTIL(SYSTEM_TIME_MS / OS_TICK_MS);

        (void)BspAdc2_UpdateAll();

        if ((offset_printed == 0u) && (BspAdc_IsCurrentOffsetReady() != 0u))
        {
            BspAdc_CalDebug_t dbg;

            BspAdc_GetCalDebug(&dbg);

            SEGGER_RTT_printf(0, "CAL_DONE,retry=%u,state=%u\r\n",
                              (unsigned)dbg.retry_count, (unsigned)dbg.state);
            SEGGER_RTT_printf(0, "CAL_A,off=%u,min=%u,max=%u,span=%u,drift=%d\r\n",
                              (unsigned)dbg.offset[0], (unsigned)dbg.min_raw[0],
                              (unsigned)dbg.max_raw[0], (unsigned)dbg.span[0],
                              (int)dbg.drift[0]);
            SEGGER_RTT_printf(0, "CAL_B,off=%u,min=%u,max=%u,span=%u,drift=%d\r\n",
                              (unsigned)dbg.offset[1], (unsigned)dbg.min_raw[1],
                              (unsigned)dbg.max_raw[1], (unsigned)dbg.span[1],
                              (int)dbg.drift[1]);
            SEGGER_RTT_printf(0, "CAL_C,off=%u,min=%u,max=%u,span=%u,drift=%d\r\n",
                              (unsigned)dbg.offset[2], (unsigned)dbg.min_raw[2],
                              (unsigned)dbg.max_raw[2], (unsigned)dbg.span[2],
                              (int)dbg.drift[2]);
            offset_printed = 1u;
        }

        ia = UserSystem_CurrentToDecimal3(BspAdc_GetIa());
        ib = UserSystem_CurrentToDecimal3(BspAdc_GetIb());
        ic = UserSystem_CurrentToDecimal3(BspAdc_GetIc());
        dq = FOC_GetDQCurrent();
        id = UserSystem_CurrentToDecimal3(dq.d);
        iq = UserSystem_CurrentToDecimal3(dq.q);
        iq_ref = UserSystem_CurrentToDecimal3(UserMotor_GetIqRef());
        iq_target = UserSystem_CurrentToDecimal3(UserMotor_GetIqRefTarget());
        fault_ia = UserSystem_CurrentToDecimal3(UserMotor_GetFaultIa());
        fault_ib = UserSystem_CurrentToDecimal3(UserMotor_GetFaultIb());
        fault_ic = UserSystem_CurrentToDecimal3(UserMotor_GetFaultIc());
        raw_a = BspAdc_GetInjectedRaw(0u);
        raw_b = BspAdc_GetInjectedRaw(1u);
        raw_c = BspAdc_GetInjectedRaw(2u);

        SEGGER_RTT_printf(0, "%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%u,%u,%u\r\n",
                          ia.sign, (unsigned)ia.integer, (unsigned)ia.fraction,
                          ib.sign, (unsigned)ib.integer, (unsigned)ib.fraction,
                          ic.sign, (unsigned)ic.integer, (unsigned)ic.fraction,
                          id.sign, (unsigned)id.integer, (unsigned)id.fraction,
                          iq.sign, (unsigned)iq.integer, (unsigned)iq.fraction,
                          iq_ref.sign, (unsigned)iq_ref.integer, (unsigned)iq_ref.fraction,
                          iq_target.sign, (unsigned)iq_target.integer, (unsigned)iq_target.fraction,
                          (unsigned)UserMotor_IsOverCurrentFault(),
                          fault_ia.sign, (unsigned)fault_ia.integer, (unsigned)fault_ia.fraction,
                          fault_ib.sign, (unsigned)fault_ib.integer, (unsigned)fault_ib.fraction,
                          fault_ic.sign, (unsigned)fault_ic.integer, (unsigned)fault_ic.fraction,
                          (unsigned)raw_a, (unsigned)raw_b, (unsigned)raw_c);
    }

    PT_END();
}
