#include "user_motor.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "user_foc.h"
#include <math.h>

#define USER_MOTOR_PI              3.14159265f
#define USER_MOTOR_TWO_PI          (2.0f * USER_MOTOR_PI)
#define USER_MOTOR_BASE_VOLTAGE    55.0f
#define USER_MOTOR_THETA_STEP_INIT 0.0015f
#define USER_MOTOR_THETA_STEP_MAX  0.006f
#define USER_MOTOR_THETA_STEP_INC  0.0000005f

static float s_open_loop_theta = 0.0f;
static float s_open_loop_step = USER_MOTOR_THETA_STEP_INIT;

static void UserMotor_UpdateOpenLoopTheta(void)
{
    if (s_open_loop_step < USER_MOTOR_THETA_STEP_MAX)
    {
        s_open_loop_step += USER_MOTOR_THETA_STEP_INC;
    }

    s_open_loop_theta += s_open_loop_step;
    if (s_open_loop_theta > USER_MOTOR_TWO_PI)
    {
        s_open_loop_theta -= USER_MOTOR_TWO_PI;
    }
}

void UserMotor_Init(void)
{
    s_open_loop_theta = 0.0f;
    s_open_loop_step = USER_MOTOR_THETA_STEP_INIT;
}

HAL_StatusTypeDef UserMotor_Start(void)
{
    HAL_StatusTypeDef status;

    status = BspAdc_StartInjected();
    if (status != HAL_OK)
    {
        return status;
    }

    return BspPwm_Start();
}

void UserMotor_FastLoop(void)
{
    float ia;
    float ib;
    float ic;
    float ua_base;
    float ub_base;
    float uc_base;
    ThreePhaseVoltage_t uabc_foc;

    UserMotor_UpdateOpenLoopTheta();

    ua_base = USER_MOTOR_BASE_VOLTAGE * sinf(s_open_loop_theta);
    ub_base = USER_MOTOR_BASE_VOLTAGE * sinf(s_open_loop_theta - 2.0f * USER_MOTOR_PI / 3.0f);
    uc_base = USER_MOTOR_BASE_VOLTAGE * sinf(s_open_loop_theta - 4.0f * USER_MOTOR_PI / 3.0f);

    ia = BspAdc_GetIa();
    ib = BspAdc_GetIb();
    ic = BspAdc_GetIc();

    FOC_CurrentLoop(ia, ib, ic, s_open_loop_theta);
    uabc_foc = FOC_GetThreePhaseVoltage();

    BspPwm_SetVoltageABC(ua_base + uabc_foc.ua,
                         ub_base + uabc_foc.ub,
                         uc_base + uabc_foc.uc);
}

float UserMotor_GetOpenLoopTheta(void)
{
    return s_open_loop_theta;
}
