/*
 * File: vf_ctrl.c
 *
 * Code generated for Simulink model 'vf_ctrl'.
 *
 * Model version                  : 2.1
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Sat Nov  1 20:37:36 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "vf_ctrl.h"
#include "VFControler.h"
#include "AntiPark.h"
#include "rtwtypes.h"
#include "vf_ctrl_private.h"

/* Exported block signals */
int16_T spd_ref;                       /* '<Root>/spd_ref' */
uint16_T mcu_ccrx[3];                  /* '<Root>/Variant Sink' */

/* Block states (default storage) */
DW_vf_ctrl_T vf_ctrl_DW;

/* Real-time model */
static RT_MODEL_vf_ctrl_T vf_ctrl_M_;
RT_MODEL_vf_ctrl_T *const vf_ctrl_M = &vf_ctrl_M_;
void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi, uint32_T
                  *ptrOutBitsLo)
{
  uint32_T absIn0;
  uint32_T absIn1;
  uint32_T in0Hi;
  uint32_T in0Lo;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  absIn0 = in0 < 0 ? ~(uint32_T)in0 + 1U : (uint32_T)in0;
  absIn1 = in1 < 0 ? ~(uint32_T)in1 + 1U : (uint32_T)in1;
  in0Hi = absIn0 >> 16U;
  in0Lo = absIn0 & 65535U;
  in1Hi = absIn1 >> 16U;
  absIn0 = absIn1 & 65535U;
  productHiLo = in0Hi * absIn0;
  productLoHi = in0Lo * in1Hi;
  absIn0 *= in0Lo;
  absIn1 = 0U;
  in0Lo = (productLoHi << /*MW:OvBitwiseOk*/ 16U) + /*MW:OvCarryOk*/ absIn0;
  if (in0Lo < absIn0) {
    absIn1 = 1U;
  }

  absIn0 = in0Lo;
  in0Lo += /*MW:OvCarryOk*/ productHiLo << /*MW:OvBitwiseOk*/ 16U;
  if (in0Lo < absIn0) {
    absIn1++;
  }

  absIn0 = (((productLoHi >> 16U) + (productHiLo >> 16U)) + in0Hi * in1Hi) +
    absIn1;
  if ((in0 != 0) && ((in1 != 0) && ((in0 > 0) != (in1 > 0)))) {
    absIn0 = ~absIn0;
    in0Lo = ~in0Lo;
    in0Lo++;
    if (in0Lo == 0U) {
      absIn0++;
    }
  }

  *ptrOutBitsHi = absIn0;
  *ptrOutBitsLo = in0Lo;
}

int32_T mul_s32_hiSR(int32_T a, int32_T b, uint32_T aShift)
{
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  return (int32_T)u32_chi >> aShift;
}

int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

int32_T div_s32(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;

    /* Divide by zero handler */
  } else {
    uint32_T tempAbsQuotient;
    tempAbsQuotient = (numerator < 0 ? ~(uint32_T)numerator + 1U : (uint32_T)
                       numerator) / (denominator < 0 ? ~(uint32_T)denominator +
      1U : (uint32_T)denominator);
    quotient = (numerator < 0) != (denominator < 0) ? -(int32_T)tempAbsQuotient :
      (int32_T)tempAbsQuotient;
  }

  return quotient;
}

/* Model step function */
void vf_ctrl_step(void)
{
  /* local block i/o variables */
  uint16_T rtb_sfun_EmSvpwm_o4;
  int16_T rtb_sfun_EmSin;
  int16_T rtb_sfun_EmCos;
  real32_T rtb_Divide1;
  int16_T rtb_DataTypeConversion;
  int16_T rtb_DataTypeConversion1;

  /* Outputs for Atomic SubSystem: '<Root>/VFControler' */

  /* Inport: '<Root>/spd_ref' */
  vf_ctrl_VFControler(spd_ref, &rtb_DataTypeConversion1, &rtb_Divide1,
                      &rtb_DataTypeConversion, &vf_ctrl_DW.VFControler);

  /* End of Outputs for SubSystem: '<Root>/VFControler' */

  /* S-Function (sfun_EmSin): '<Root>/sfun_EmSin' */
  rtb_sfun_EmSin = EmSin(rtb_DataTypeConversion);

  /* S-Function (sfun_EmCos): '<Root>/sfun_EmCos' */
  rtb_sfun_EmCos = EmCos(rtb_DataTypeConversion);

  /* Outputs for Atomic SubSystem: '<Root>/AntiPark' */
  vf_ctrl_AntiPark(vf_ctrl_ConstB.VFControler.Constant, rtb_DataTypeConversion1,
                   rtb_sfun_EmSin, rtb_sfun_EmCos, &rtb_DataTypeConversion,
                   &rtb_DataTypeConversion1);

  /* End of Outputs for SubSystem: '<Root>/AntiPark' */

  /* SignalConversion generated from: '<Root>/Variant Sink' incorporates:
   *  S-Function (sfun_EmSvpwm): '<Root>/sfun_EmSvpwm'
   */
  EmSvpwm(rtb_DataTypeConversion, rtb_DataTypeConversion1, &(&mcu_ccrx[0])[0],
          &mcu_ccrx[1], &mcu_ccrx[2], &rtb_sfun_EmSvpwm_o4);
}

/* Model initialize function */
void vf_ctrl_initialize(void)
{
  /* Registration code */

  /* external inputs */
  spd_ref = 200;
}

/* Model terminate function */
void vf_ctrl_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
