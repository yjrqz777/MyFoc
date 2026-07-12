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
    SEGGER_RTT_WriteString(0, "Ia,Ib,Ic,Id,Iq,s_iq_ref,iq_ref_target,oc_fault,fault_ia,fault_ib,fault_ic\r\n");
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

        PT_WAIT_UNTIL(SYSTEM_TIME_MS / OS_TICK_MS);

        (void)BspAdc2_UpdateAll();

        if ((offset_printed == 0u) && (BspAdc_IsCurrentOffsetReady() != 0u))
        {
            tCurrentDecimal3 off_a = UserSystem_CurrentToDecimal3(BspAdc_GetCurrentOffsetVoltage(0u));
            tCurrentDecimal3 off_b = UserSystem_CurrentToDecimal3(BspAdc_GetCurrentOffsetVoltage(1u));
            tCurrentDecimal3 off_c = UserSystem_CurrentToDecimal3(BspAdc_GetCurrentOffsetVoltage(2u));
            tCurrentDecimal3 off_bus = UserSystem_CurrentToDecimal3(BspAdc_GetCurrentOffsetVoltage(3u));
            uint16_t raw_a = (uint16_t)(BspAdc_GetCurrentOffsetRaw(0u) + 0.5f);
            uint16_t raw_b = (uint16_t)(BspAdc_GetCurrentOffsetRaw(1u) + 0.5f);
            uint16_t raw_c = (uint16_t)(BspAdc_GetCurrentOffsetRaw(2u) + 0.5f);
            uint16_t raw_bus = (uint16_t)(BspAdc_GetCurrentOffsetRaw(3u) + 0.5f);

            SEGGER_RTT_printf(0, "ADC_OFFSET_V,rawA=%u,rawB=%u,rawC=%u,rawBus=%u,VA=%s%u.%03u,VB=%s%u.%03u,VC=%s%u.%03u,VBusOff=%s%u.%03u\r\n",
                              (unsigned)raw_a, (unsigned)raw_b, (unsigned)raw_c, (unsigned)raw_bus,
                              off_a.sign, (unsigned)off_a.integer, (unsigned)off_a.fraction,
                              off_b.sign, (unsigned)off_b.integer, (unsigned)off_b.fraction,
                              off_c.sign, (unsigned)off_c.integer, (unsigned)off_c.fraction,
                              off_bus.sign, (unsigned)off_bus.integer, (unsigned)off_bus.fraction);
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

        SEGGER_RTT_printf(0, "%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%u,%s%u.%03u,%s%u.%03u,%s%u.%03u\r\n",
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
                          fault_ic.sign, (unsigned)fault_ic.integer, (unsigned)fault_ic.fraction);
    }

    PT_END();
}
