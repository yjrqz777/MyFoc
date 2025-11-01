/*
 * File: AntiPark.c
 *
 * Code generated for Simulink model 'vf_ctrl'.
 *
 * Model version                  : 2.1
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Sat Nov  1 20:50:31 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "rtwtypes.h"
#include "AntiPark.h"

/* Output and update for atomic system: '<Root>/AntiPark' */
void vf_ctrl_AntiPark(int16_T rtu_d, int16_T rtu_q, int16_T rtu_sin, int16_T
                      rtu_cos, int16_T *rty_u, int16_T *rty_u_j)
{
  int32_T u0;

  /* ArithShift: '<S1>/Shift Arithmetic' incorporates:
   *  Constant: '<S1>/Constant'
   *  Product: '<S1>/Product'
   *  Product: '<S1>/Product1'
   *  Sum: '<S1>/Add'
   */
  u0 = (rtu_d * rtu_cos - rtu_q * rtu_sin) >> 15U;

  /* Saturate: '<S1>/Saturation' */
  if (u0 > 32767) {
    /* DataTypeConversion: '<S1>/Data Type Conversion' */
    *rty_u = MAX_int16_T;
  } else if (u0 < -32768) {
    /* DataTypeConversion: '<S1>/Data Type Conversion' */
    *rty_u = MIN_int16_T;
  } else {
    /* DataTypeConversion: '<S1>/Data Type Conversion' */
    *rty_u = (int16_T)u0;
  }

  /* End of Saturate: '<S1>/Saturation' */

  /* ArithShift: '<S1>/Shift Arithmetic1' incorporates:
   *  Constant: '<S1>/Constant1'
   *  Product: '<S1>/Product2'
   *  Product: '<S1>/Product3'
   *  Sum: '<S1>/Add1'
   */
  u0 = (rtu_d * rtu_sin + rtu_q * rtu_cos) >> 15U;

  /* Saturate: '<S1>/Saturation1' */
  if (u0 > 32767) {
    /* DataTypeConversion: '<S1>/Data Type Conversion1' */
    *rty_u_j = MAX_int16_T;
  } else if (u0 < -32768) {
    /* DataTypeConversion: '<S1>/Data Type Conversion1' */
    *rty_u_j = MIN_int16_T;
  } else {
    /* DataTypeConversion: '<S1>/Data Type Conversion1' */
    *rty_u_j = (int16_T)u0;
  }

  /* End of Saturate: '<S1>/Saturation1' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
