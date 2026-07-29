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
#include "bsp_hall.h"
#include "user_foc.h"
#include "user_motor.h"
tSysDataDef tSysData;

/** @brief 三位小数电流格式化结构体（用于 RTT CSV 输出） */
typedef struct tCurrentDecimal3Def
{
    const char * pcSign;      /**< 符号串，"" 或 "-" */
    uint32_t u32Integer;      /**< 整数部分（A） */
    uint32_t u32Fraction;     /**< 小数部分（毫安，千分位） */
} tCurrentDecimal3Def;

/**
 * @brief  将电流值（A）转换为毫安整数（四舍五入）
 * @param[in] f32Current  电流值，单位 A
 * @return 电流的毫安等效值（正数向上取整，负数向下取整）
 * @note   用于 RTT 输出的格式化转换
 */
static int32_t UsrSystemCurrentToMilliAmp(float f32Current)
{
    if (f32Current >= 0.0f)
    {
        return (int32_t)((f32Current * 1000.0f) + 0.5f);
    }

    return (int32_t)((f32Current * 1000.0f) - 0.5f);
}

/**
 * @brief  将电流值（A）转换为带符号和小数部分的十进制结构体
 * @param[in] f32Current  电流值，单位 A
 * @return tCurrentDecimal3Def 结构体，包含符号串、整数部分和小数部分（千分位）
 * @note   用于 RTT CSV 格式输出，格式如 "-1.234" 或 "0.567"
 */
static tCurrentDecimal3Def UsrSystemCurrentToDecimal3(float f32Current)
{
    int32_t MilliAmp = UsrSystemCurrentToMilliAmp(f32Current);
    uint32_t AbsoluteMilliAmp;
    tCurrentDecimal3Def Decimal;

    if (MilliAmp < 0)
    {
        AbsoluteMilliAmp = (uint32_t)(-MilliAmp);
        Decimal.pcSign = "-";
    }
    else
    {
        AbsoluteMilliAmp = (uint32_t)MilliAmp;
        Decimal.pcSign = "";
    }

    Decimal.u32Integer = AbsoluteMilliAmp / 1000u;
    Decimal.u32Fraction = AbsoluteMilliAmp % 1000u;

    return Decimal;
}


/**
 * @brief  初始化系统状态和数据
 * @note   设置系统状态为 E_SYS_STATE_INIT，清零运行计数，
 *         并通过 RTT 输出电流 CSV 格式标题行，供 J-Scope 数据采集
 */
void UsrSystemInit(void)
{
    tSysData.eState = E_SYS_STATE_INIT;
    tSysData.u32PowerOnTimes = 0u;
    tSysData.u32OpenTimes = 0u;
    SEGGER_RTT_WriteString(0, "Ia,Ib,Ic,Id,Iq,s_iq_ref,iq_ref_target,oc_fault,fault_ia,fault_ib,fault_ic,rawA,rawB,rawC\r\n");
}


/**
 * @brief  Protothread 系统协程任务 — 周期性采集并输出电流数据
 * @return PT 状态码
 * @note   首次进入时初始化系统，之后按 USR_SYSTEM_TASK_INTERVAL_MS 周期执行：
 *         - 轮询更新 ADC2 通道值（母线电压、电位器等）
 *         - 采集三相电流 (Ia/Ib/Ic) 和 dq 轴电流 (Id/Iq)
 *         - 通过 RTT 输出 CSV 格式数据流，供上位机或 J-Scope 采集
 */
uint16_t UsrSystemTask(void)
{
    static uint8_t u8OffsetPrinted = 0u;

    PT_BEGIN()
    {
        UsrSystemInit();
    }

    while (1)
    {
        tCurrentDecimal3Def PhaseACurrent;
        tCurrentDecimal3Def PhaseBCurrent;
        tCurrentDecimal3Def PhaseCCurrent;
        tCurrentDecimal3Def DirectCurrent;
        tCurrentDecimal3Def QuadratureCurrent;
        tCurrentDecimal3Def QuadratureReference;
        tCurrentDecimal3Def QuadratureTarget;
        tCurrentDecimal3Def FaultPhaseACurrent;
        tCurrentDecimal3Def FaultPhaseBCurrent;
        tCurrentDecimal3Def FaultPhaseCCurrent;
        tDqCurrentDef DqCurrent;
        uint16_t RawPhaseA;
        uint16_t RawPhaseB;
        uint16_t RawPhaseC;

        PT_WAIT_UNTIL(USR_SYSTEM_TASK_INTERVAL_MS / OS_TICK_MS);

        (void)BspAdc2UpdateAll();

        if ((u8OffsetPrinted == 0u) && (BspAdcIsCurrentOffsetReady() != 0u))
        {
            tBspAdcCalDebugDef CalibrationDebug;

            BspAdcGetCalDebug(&CalibrationDebug);

            SEGGER_RTT_printf(0, "CAL_DONE,retry=%u,state=%u\r\n",
                              (unsigned)CalibrationDebug.u8RetryCount, (unsigned)CalibrationDebug.eState);
            SEGGER_RTT_printf(0, "CAL_A,off=%u,min=%u,max=%u,span=%u,drift=%d\r\n",
                              (unsigned)CalibrationDebug.u16Offset[0], (unsigned)CalibrationDebug.u16MinRaw[0],
                              (unsigned)CalibrationDebug.u16MaxRaw[0], (unsigned)CalibrationDebug.u16Span[0],
                              (int)CalibrationDebug.s16Drift[0]);
            SEGGER_RTT_printf(0, "CAL_B,off=%u,min=%u,max=%u,span=%u,drift=%d\r\n",
                              (unsigned)CalibrationDebug.u16Offset[1], (unsigned)CalibrationDebug.u16MinRaw[1],
                              (unsigned)CalibrationDebug.u16MaxRaw[1], (unsigned)CalibrationDebug.u16Span[1],
                              (int)CalibrationDebug.s16Drift[1]);
            SEGGER_RTT_printf(0, "CAL_C,off=%u,min=%u,max=%u,span=%u,drift=%d\r\n",
                              (unsigned)CalibrationDebug.u16Offset[2], (unsigned)CalibrationDebug.u16MinRaw[2],
                              (unsigned)CalibrationDebug.u16MaxRaw[2], (unsigned)CalibrationDebug.u16Span[2],
                              (int)CalibrationDebug.s16Drift[2]);
            u8OffsetPrinted = 1u;
        }

        PhaseACurrent = UsrSystemCurrentToDecimal3(BspAdcGetIa());
        PhaseBCurrent = UsrSystemCurrentToDecimal3(BspAdcGetIb());
        PhaseCCurrent = UsrSystemCurrentToDecimal3(BspAdcGetIc());
        DqCurrent = UsrFocGetDqCurrent();
        DirectCurrent = UsrSystemCurrentToDecimal3(DqCurrent.f32D);
        QuadratureCurrent = UsrSystemCurrentToDecimal3(DqCurrent.f32Q);
        QuadratureReference = UsrSystemCurrentToDecimal3(UsrMotorGetIqRef());
        QuadratureTarget = UsrSystemCurrentToDecimal3(UsrMotorGetIqRefTarget());
        FaultPhaseACurrent = UsrSystemCurrentToDecimal3(UsrMotorGetFaultIa());
        FaultPhaseBCurrent = UsrSystemCurrentToDecimal3(UsrMotorGetFaultIb());
        FaultPhaseCCurrent = UsrSystemCurrentToDecimal3(UsrMotorGetFaultIc());
        RawPhaseA = BspAdcGetInjectedRaw(0u);
        RawPhaseB = BspAdcGetInjectedRaw(1u);
        RawPhaseC = BspAdcGetInjectedRaw(2u);

        SEGGER_RTT_printf(0, "%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%u,%s%u.%03u,%s%u.%03u,%s%u.%03u,%u,%u,%u\r\n",
                          PhaseACurrent.pcSign, (unsigned)PhaseACurrent.u32Integer, (unsigned)PhaseACurrent.u32Fraction,
                          PhaseBCurrent.pcSign, (unsigned)PhaseBCurrent.u32Integer, (unsigned)PhaseBCurrent.u32Fraction,
                          PhaseCCurrent.pcSign, (unsigned)PhaseCCurrent.u32Integer, (unsigned)PhaseCCurrent.u32Fraction,
                          DirectCurrent.pcSign, (unsigned)DirectCurrent.u32Integer, (unsigned)DirectCurrent.u32Fraction,
                          QuadratureCurrent.pcSign, (unsigned)QuadratureCurrent.u32Integer, (unsigned)QuadratureCurrent.u32Fraction,
                          QuadratureReference.pcSign, (unsigned)QuadratureReference.u32Integer, (unsigned)QuadratureReference.u32Fraction,
                          QuadratureTarget.pcSign, (unsigned)QuadratureTarget.u32Integer, (unsigned)QuadratureTarget.u32Fraction,
                          (unsigned)UsrMotorIsOverCurrentFault(),
                          FaultPhaseACurrent.pcSign, (unsigned)FaultPhaseACurrent.u32Integer, (unsigned)FaultPhaseACurrent.u32Fraction,
                          FaultPhaseBCurrent.pcSign, (unsigned)FaultPhaseBCurrent.u32Integer, (unsigned)FaultPhaseBCurrent.u32Fraction,
                          FaultPhaseCCurrent.pcSign, (unsigned)FaultPhaseCCurrent.u32Integer, (unsigned)FaultPhaseCCurrent.u32Fraction,
                          (unsigned)RawPhaseA, (unsigned)RawPhaseB, (unsigned)RawPhaseC);

        SEGGER_RTT_printf(0, "HALL,state=%u,valid=%u,angle=%d,speed=%d\r\n",
                          (unsigned)BspHallGetState(),
                          (unsigned)BspHallIsAngleValid(),
                          (int)(BspHallGetElectricalAngle() * 57.2958f),
                          (int)(BspHallGetElectricalSpeed() * 1000.0f));
    }

    PT_END();
}
