/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2025-05-11 15:37:03
 * LastEditTime: 2026-03-18 19:41:53
 * LastEditors: yjrqz777 3210551161@qq.com
 * Description:
 * FilePath: \g474app\My_Lib\FOC\foc.c
 * @YJRQZ777
***************************************************************************************************/

#include "foc.h"
#include <math.h>

#define FOC_ADC_AVG_SAMPLES      (10U)
#define FOC_POLE_PAIRS           (7)
#define FOC_FALLBACK_TS_S        (1e-4f)
#define FOC_MAX_TS_S             (0.5f)
#define FOC_MIN_SUPPLY_VOLTAGE   (1e-3f)
#define FOC_TWO_PI               (2.0f * PI)
#define FOC_SQRT3                (1.7320508075688772f)
#define FOC_VF_VOLTAGE_BOOST_V   (0.6f)
#define FOC_VF_MAX_ELEC_FREQ_HZ  (180.0f)
#define FOC_VF_MAX_UQ_RATIO      (0.45f)
#define FOC_VF_ACCEL_RAD_S2      (300.0f)
#define FOC_VF_UQ_FILTER_BW_HZ   (60.0f)
#define FOC_VF_ZERO_SPEED_EPS    (1e-2f)

Foc_DataDef Foc_Data = {0};

float voltage_power_supply = 5.0f;
float shaft_angle = 0.0f;
float zero_electric_angle = 0.0f;
float Ualpha = 0.0f;
float Ubeta = 0.0f;
float Ua = 0.0f;
float Ub = 0.0f;
float Uc = 0.0f;
float dc_a = 0.0f;
float dc_b = 0.0f;
float dc_c = 0.0f;

static uint32_t s_open_loop_timestamp_ms = 0U;
static float s_vf_velocity_cmd_rad_s = 0.0f;
static float s_vf_uq_voltage_v = 0.0f;

static inline float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static inline float electricalAngle(float mech_angle, int pole_pairs)
{
    return mech_angle * (float)pole_pairs;
}

static inline float lowPassFilter(float input, float prev, float bandwidth_hz, float ts_s)
{
    if ((bandwidth_hz <= 0.0f) || (ts_s <= 0.0f)) {
        return input;
    }

    const float tau = 1.0f / (FOC_TWO_PI * bandwidth_hz);
    const float alpha = ts_s / (tau + ts_s);
    return prev + alpha * (input - prev);
}

static inline float normalizeAngle(float angle)
{
    float wrapped = fmodf(angle, FOC_TWO_PI);
    return (wrapped >= 0.0f) ? wrapped : (wrapped + FOC_TWO_PI);
}

static void setPwm(float ua, float ub, float uc)
{
    const float safe_supply = (voltage_power_supply > FOC_MIN_SUPPLY_VOLTAGE)
                                  ? voltage_power_supply
                                  : FOC_MIN_SUPPLY_VOLTAGE;
    const float inv_supply = 1.0f / safe_supply;
    const uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim1);

    dc_a = clampf(ua * inv_supply, 0.0f, 1.0f);
    dc_b = clampf(ub * inv_supply, 0.0f, 1.0f);
    dc_c = clampf(uc * inv_supply, 0.0f, 1.0f);

    uint32_t ccr1 = (uint32_t)(dc_a * (float)pwm_period + 0.5f);
    uint32_t ccr2 = (uint32_t)(dc_b * (float)pwm_period + 0.5f);
    uint32_t ccr3 = (uint32_t)(dc_c * (float)pwm_period + 0.5f);

    if (ccr1 > pwm_period) {
        ccr1 = pwm_period;
    }
    if (ccr2 > pwm_period) {
        ccr2 = pwm_period;
    }
    if (ccr3 > pwm_period) {
        ccr3 = pwm_period;
    }

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, ccr3);
}

static void setPhaseVoltage(float uq, float ud, float angle_el)
{
    const float half_bus = voltage_power_supply * 0.5f;
    const float max_voltage = half_bus * 0.98f;
    float sin_a;
    float cos_a;
    const float u_norm = sqrtf(uq * uq + ud * ud);

    if ((u_norm > max_voltage) && (u_norm > 0.0f)) {
        const float scale = max_voltage / u_norm;
        uq *= scale;
        ud *= scale;
    }

    angle_el = normalizeAngle(angle_el + zero_electric_angle);
    sin_a = sinf(angle_el);
    cos_a = cosf(angle_el);

    Ualpha = ud * cos_a - uq * sin_a;
    Ubeta = uq * cos_a + ud * sin_a;

    Ua = Ualpha + half_bus;
    Ub = (FOC_SQRT3 * Ubeta - Ualpha) * 0.5f + half_bus;
    Uc = (-Ualpha - FOC_SQRT3 * Ubeta) * 0.5f + half_bus;

    setPwm(Ua, Ub, Uc);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint32_t u32Ia = 0U;
    static uint32_t u32Ib = 0U;
    static uint32_t u32Ic = 0U;
    static uint32_t u32Ibus = 0U;
    static uint8_t u8count = 0U;

    if ((hadc == NULL) || (hadc->Instance != ADC1)) {
        return;
    }

    u32Ia += HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    u32Ib += HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    u32Ic += HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    u32Ibus += HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
    u8count++;

    if (u8count >= FOC_ADC_AVG_SAMPLES) {
        Foc_Data.I_Def.Ia = u32Ia / FOC_ADC_AVG_SAMPLES;
        Foc_Data.I_Def.Ib = u32Ib / FOC_ADC_AVG_SAMPLES;
        Foc_Data.I_Def.Ic = u32Ic / FOC_ADC_AVG_SAMPLES;
        Foc_Data.I_Def.Ibus = u32Ibus / FOC_ADC_AVG_SAMPLES;

        u32Ia = 0U;
        u32Ib = 0U;
        u32Ic = 0U;
        u32Ibus = 0U;
        u8count = 0U;

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
}

int PT_TASK_Test(void)
{
    PT_BEGIN()
    {
    }
    while (1)
    {
        PT_WAIT_UNTIL(10 / TIME_ms);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    PT_END();
}

uint16_t ADC_Read(ADC_HandleTypeDef AdcNum, uint32_t Channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t u16adcValue = 0U;

    sConfig.Channel = Channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&AdcNum, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADC_Start(&AdcNum) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_PollForConversion(&AdcNum, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }

    u16adcValue = (uint16_t)HAL_ADC_GetValue(&AdcNum);
    (void)HAL_ADC_Stop(&AdcNum);

    return u16adcValue;
}

float velocityOpenloop(float target_velocity)
{
    const uint32_t now_ms = HAL_GetTick();
    float Ts = FOC_FALLBACK_TS_S;
    float max_speed_step;
    float speed_delta;
    float elec_freq_hz;
    float uq_ref;
    float uq_limit;
    float vf_slope;

    if (s_open_loop_timestamp_ms != 0U) {
        Ts = (float)(now_ms - s_open_loop_timestamp_ms) * 1e-3f;
        if ((Ts <= 0.0f) || (Ts > FOC_MAX_TS_S)) {
            Ts = FOC_FALLBACK_TS_S;
        }
    }
    s_open_loop_timestamp_ms = now_ms;

    max_speed_step = FOC_VF_ACCEL_RAD_S2 * Ts;
    speed_delta = target_velocity - s_vf_velocity_cmd_rad_s;
    speed_delta = clampf(speed_delta, -max_speed_step, max_speed_step);
    s_vf_velocity_cmd_rad_s += speed_delta;

    shaft_angle = normalizeAngle(shaft_angle + s_vf_velocity_cmd_rad_s * Ts);

    elec_freq_hz = fabsf(s_vf_velocity_cmd_rad_s) * ((float)FOC_POLE_PAIRS / FOC_TWO_PI);
    elec_freq_hz = clampf(elec_freq_hz, 0.0f, FOC_VF_MAX_ELEC_FREQ_HZ);

    uq_limit = voltage_power_supply * FOC_VF_MAX_UQ_RATIO;
    uq_limit = clampf(uq_limit, 0.0f, voltage_power_supply * 0.5f * 0.98f);
    vf_slope = (FOC_VF_MAX_ELEC_FREQ_HZ > 0.0f) ? (uq_limit / FOC_VF_MAX_ELEC_FREQ_HZ) : 0.0f;

    if (elec_freq_hz < FOC_VF_ZERO_SPEED_EPS) {
        uq_ref = 0.0f;
    } else {
        uq_ref = FOC_VF_VOLTAGE_BOOST_V + vf_slope * elec_freq_hz;
    }
    uq_ref = clampf(uq_ref, 0.0f, uq_limit);

    s_vf_uq_voltage_v = lowPassFilter(uq_ref, s_vf_uq_voltage_v, FOC_VF_UQ_FILTER_BW_HZ, Ts);
    setPhaseVoltage(s_vf_uq_voltage_v, 0.0f, electricalAngle(shaft_angle, FOC_POLE_PAIRS));

    return s_vf_uq_voltage_v;
}

void FocInit(void)
{
    s_open_loop_timestamp_ms = HAL_GetTick();
    s_vf_velocity_cmd_rad_s = 0.0f;
    s_vf_uq_voltage_v = 0.0f;
    shaft_angle = 0.0f;
}

int PT_TASK_FOC(void)
{
    static uint8_t print_div = 0U;

    PT_BEGIN()
    {
        FocInit();
        // printf("%d\n", 222);
    }
    while (1)
    {
        float target_velocity = 0.0f;
        float uq = 0.0f;

        PT_WAIT_UNTIL(1);
        target_velocity = (((float)ADC_Read(hadc2, ADC_CHANNEL_5) / 4095.0f) - 0.5f) * 2.0f * 80.0f;
        uq = velocityOpenloop(target_velocity);

        // print_div++;
        // if (print_div >= 10U) {
        //     print_div = 0U;
        //     printf("%.2f,%.2f,%.3f,%.3f,%.3f\n",target_velocity, uq, dc_a, dc_b, dc_c);
        // }
    }
    PT_END();
}
