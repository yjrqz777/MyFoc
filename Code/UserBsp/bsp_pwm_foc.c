#include "bsp_pwm_foc.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"

/**
 * @brief   Q10 占空比换算为 TIM1 比较寄存器值
 * @param[in] u16Duty    Q10 格式占空比，上限钳位到 BSP_PWM_DUTY_Q10_MAX
 * @param[in] u16Period  定时器周期值
 * @return  比较寄存器 CCR 值
 * @note    TIM1 采用 PWM 模式 2，占空比增大时 CCR 减小；输出钳位到
 *          [CS_CCR_MIN, u16Period]
 */
static uint16_t BspPwmDutyQ10ToCompare(uint16_t u16Duty, uint16_t u16Period)
{
    uint32_t u32Compare;

    if (u16Duty > BSP_PWM_DUTY_Q10_MAX)
    {
        u16Duty = BSP_PWM_DUTY_Q10_MAX;
    }

    /* TIM1 uses PWM mode 2: high-side duty increases as CCR decreases. */
    u32Compare = (uint32_t)u16Period -
                 (((uint32_t)u16Duty * (uint32_t)u16Period) /
                  BSP_PWM_DUTY_Q10_MAX);
    if (u32Compare < CS_CCR_MIN)
    {
        u32Compare = CS_CCR_MIN;
    }
    if (u32Compare > u16Period)
    {
        u32Compare = u16Period;
    }
    return (uint16_t)u32Compare;
}

/**
 * @brief   设置三相 PWM 占空比
 * @param[in] u16DutyA  A 相占空比(Q10)
 * @param[in] u16DutyB  B 相占空比(Q10)
 * @param[in] u16DutyC  C 相占空比(Q10)
 * @note    逐相换算为比较寄存器值后写入三路通道
 */
void BspPwmSetDutyQ10(uint16_t u16DutyA,
                      uint16_t u16DutyB,
                      uint16_t u16DutyC)
{
    uint16_t u16Period = BspPwmGetPeriod();

    BspPwmSetCompare(BspPwmDutyQ10ToCompare(u16DutyA, u16Period),
                     BspPwmDutyQ10ToCompare(u16DutyB, u16Period),
                     BspPwmDutyQ10ToCompare(u16DutyC, u16Period));
}
