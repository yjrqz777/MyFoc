/*
 * File: AntiPark.h
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

#ifndef RTW_HEADER_AntiPark_h_
#define RTW_HEADER_AntiPark_h_
#ifndef vf_ctrl_COMMON_INCLUDES_
#define vf_ctrl_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "em_sincos.h"
#include "em_svpwm.h"
#endif                                 /* vf_ctrl_COMMON_INCLUDES_ */

extern void vf_ctrl_AntiPark(int16_T rtu_d, int16_T rtu_q, int16_T rtu_sin,
  int16_T rtu_cos, int16_T *rty_u, int16_T *rty_u_j);

#endif                                 /* RTW_HEADER_AntiPark_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
