/*
 * File: vf_ctrl.h
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

#ifndef RTW_HEADER_vf_ctrl_h_
#define RTW_HEADER_vf_ctrl_h_
#ifndef vf_ctrl_COMMON_INCLUDES_
#define vf_ctrl_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "em_sincos.h"
#include "em_svpwm.h"
#endif                                 /* vf_ctrl_COMMON_INCLUDES_ */

#include "vf_ctrl_types.h"
#include "VFControler.h"

/* Includes for objects with custom storage classes */
#include "model_param.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  DW_VFControler_vf_ctrl_T VFControler;/* '<Root>/VFControler' */
} DW_vf_ctrl_T;

/* Invariant block signals (default storage) */
typedef struct {
  ConstB_VFControler_vf_ctrl_T VFControler;/* '<Root>/VFControler' */
} ConstB_vf_ctrl_T;

/* Real-time Model Data Structure */
struct tag_RTM_vf_ctrl_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_vf_ctrl_T vf_ctrl_DW;
extern const ConstB_vf_ctrl_T vf_ctrl_ConstB;/* constant block i/o */

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern int16_T spd_ref;                /* '<Root>/spd_ref' */
extern uint16_T mcu_ccrx[3];           /* '<Root>/Variant Sink' */

/* Model entry point functions */
extern void vf_ctrl_initialize(void);
extern void vf_ctrl_step(void);
extern void vf_ctrl_terminate(void);

/* Real-time Model object */
extern RT_MODEL_vf_ctrl_T *const vf_ctrl_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<Root>/Scope9' : Unused code path elimination
 * Block '<Root>/Scope' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'vf_ctrl'
 * '<S1>'   : 'vf_ctrl/AntiPark'
 * '<S2>'   : 'vf_ctrl/VFControler'
 * '<S3>'   : 'vf_ctrl/VFControler/SpdToVolJSF630'
 * '<S4>'   : 'vf_ctrl/VFControler/SpdToVolTB2P'
 * '<S5>'   : 'vf_ctrl/VFControler/SpdToVolTG5P'
 * '<S6>'   : 'vf_ctrl/VFControler/SpdToVolUnDef'
 * '<S7>'   : 'vf_ctrl/VFControler/SpdToVolJSF630/SpdToVol'
 * '<S8>'   : 'vf_ctrl/VFControler/SpdToVolTB2P/SpdToVol'
 * '<S9>'   : 'vf_ctrl/VFControler/SpdToVolTG5P/SpdToVol'
 */
#endif                                 /* RTW_HEADER_vf_ctrl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
