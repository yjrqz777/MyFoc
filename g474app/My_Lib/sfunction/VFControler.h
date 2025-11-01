/*
 * File: VFControler.h
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

#ifndef RTW_HEADER_VFControler_h_
#define RTW_HEADER_VFControler_h_
#ifndef vf_ctrl_COMMON_INCLUDES_
#define vf_ctrl_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "em_sincos.h"
#include "em_svpwm.h"
#endif                                 /* vf_ctrl_COMMON_INCLUDES_ */

/* Block states (default storage) for system '<Root>/VFControler' */
typedef struct {
  int32_T DiscreteTimeIntegrator_DSTATE;/* '<S2>/Discrete-Time Integrator' */
} DW_VFControler_vf_ctrl_T;

/* Invariant block signals for system '<Root>/VFControler' */
typedef struct {
  const int16_T Constant;              /* '<S2>/Constant' */
} ConstB_VFControler_vf_ctrl_T;

extern void vf_ctrl_VFControler(int16_T rtu_spd_ref, int16_T *rty_uq_ref,
  real32_T *rty_e_ref, int16_T *rty_e_ref_l, DW_VFControler_vf_ctrl_T *localDW);

#endif                                 /* RTW_HEADER_VFControler_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
