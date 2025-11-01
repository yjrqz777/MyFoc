/*
 * File: VFControler.c
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

#include "rtwtypes.h"
#include "VFControler.h"
#include "vf_ctrl_private.h"
#include "model_param.h"

/* Output and update for atomic system: '<Root>/VFControler' */
void vf_ctrl_VFControler(int16_T rtu_spd_ref, int16_T *rty_uq_ref, real32_T
  *rty_e_ref, int16_T *rty_e_ref_l, DW_VFControler_vf_ctrl_T *localDW)
{
  int16_T rtb_Saturation1_c;

  /* Saturate: '<S2>/Saturation1' */
  if (rtu_spd_ref > 600) {
    rtb_Saturation1_c = 600;
  } else if (rtu_spd_ref < -600) {
    rtb_Saturation1_c = -600;
  } else {
    rtb_Saturation1_c = rtu_spd_ref;
  }

  /* End of Saturate: '<S2>/Saturation1' */

  /* Product: '<S2>/Divide1' incorporates:
   *  Constant: '<S2>/Constant1'
   *  Constant: '<S2>/Constant3'
   *  Constant: '<S2>/Constant4'
   *  Product: '<S2>/Product'
   *  Product: '<S2>/Product3'
   */
  *rty_e_ref = (real32_T)rtb_Saturation1_c * 6.28318548F * (real32_T)((uint8_T)
    MOTOR_PN) / 60.0F;

  /* SwitchCase: '<S2>/Switch Case' incorporates:
   *  Constant: '<S2>/Constant5'
   */
  switch (((uint8_T)MOTOR_TYPE)) {
   case 11:
    {
      int16_T u_abs;

      /* Outputs for IfAction SubSystem: '<S2>/SpdToVolTB2P' incorporates:
       *  ActionPort: '<S4>/Action Port'
       */
      /* MATLAB Function: '<S4>/SpdToVol' */
      u_abs = rtb_Saturation1_c;
      if (rtb_Saturation1_c < 0) {
        u_abs = (int16_T)-rtb_Saturation1_c;
      }

      if (u_abs == 0) {
        u_abs = 0;
      } else if (u_abs <= 100) {
        u_abs = 2955;
      } else if (u_abs <= 200) {
        u_abs = 3547;
      } else if (u_abs <= 300) {
        u_abs = 4138;
      } else if (u_abs <= 400) {
        u_abs = 4729;
      } else if (u_abs <= 500) {
        u_abs = 5320;
      } else if (u_abs <= 550) {
        u_abs = 5911;
      } else {
        u_abs = 6503;
      }

      if (rtb_Saturation1_c < 0) {
        u_abs = (int16_T)-u_abs;
      }

      /* End of MATLAB Function: '<S4>/SpdToVol' */

      /* Saturate: '<S2>/Saturation' incorporates:
       *  SignalConversion generated from: '<S4>/Out1'
       */
      *rty_uq_ref = u_abs;

      /* End of Outputs for SubSystem: '<S2>/SpdToVolTB2P' */
    }
    break;

   case 3:
    {
      int16_T u_abs;

      /* Outputs for IfAction SubSystem: '<S2>/SpdToVolTG5P' incorporates:
       *  ActionPort: '<S5>/Action Port'
       */
      /* MATLAB Function: '<S5>/SpdToVol' */
      u_abs = rtb_Saturation1_c;
      if (rtb_Saturation1_c < 0) {
        u_abs = (int16_T)-rtb_Saturation1_c;
      }

      if (u_abs == 0) {
        u_abs = 0;
      } else if (u_abs <= 200) {
        u_abs = 3700;
      } else if (u_abs <= 300) {
        u_abs = 4300;
      } else if (u_abs <= 350) {
        u_abs = 4900;
      } else if (u_abs <= 450) {
        u_abs = 5500;
      } else {
        u_abs = 6503;
      }

      if (rtb_Saturation1_c < 0) {
        u_abs = (int16_T)-u_abs;
      }

      /* End of MATLAB Function: '<S5>/SpdToVol' */

      /* Saturate: '<S2>/Saturation' incorporates:
       *  SignalConversion generated from: '<S5>/Out1'
       */
      *rty_uq_ref = u_abs;

      /* End of Outputs for SubSystem: '<S2>/SpdToVolTG5P' */
    }
    break;

   case 21:
    {
      int16_T u_abs;

      /* Outputs for IfAction SubSystem: '<S2>/SpdToVolJSF630' incorporates:
       *  ActionPort: '<S3>/Action Port'
       */
      /* MATLAB Function: '<S3>/SpdToVol' */
      u_abs = rtb_Saturation1_c;
      if (rtb_Saturation1_c < 0) {
        u_abs = (int16_T)-rtb_Saturation1_c;
      }

      if (u_abs == 0) {
        u_abs = 0;
      } else if (u_abs <= 100) {
        u_abs = 3277;
      } else if (u_abs <= 300) {
        u_abs = 4916;
      } else if (u_abs <= 400) {
        u_abs = 5911;
      } else if (u_abs <= 500) {
        u_abs = 6503;
      } else {
        u_abs = 7500;
      }

      if (rtb_Saturation1_c < 0) {
        u_abs = (int16_T)-u_abs;
      }

      /* End of MATLAB Function: '<S3>/SpdToVol' */

      /* Saturate: '<S2>/Saturation' incorporates:
       *  SignalConversion generated from: '<S3>/Out1'
       */
      *rty_uq_ref = u_abs;

      /* End of Outputs for SubSystem: '<S2>/SpdToVolJSF630' */
    }
    break;

   default:
    /* Outputs for IfAction SubSystem: '<S2>/SpdToVolUnDef' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    /* Saturate: '<S2>/Saturation' incorporates:
     *  Constant: '<S6>/Constant'
     *  SignalConversion generated from: '<S6>/Out1'
     */
    *rty_uq_ref = 0;

    /* End of Outputs for SubSystem: '<S2>/SpdToVolUnDef' */
    break;
  }

  /* End of SwitchCase: '<S2>/Switch Case' */

  /* DataTypeConversion: '<S2>/Data Type Conversion' incorporates:
   *  DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
   *  Math: '<S2>/Math Function'
   */
  *rty_e_ref_l = (int16_T)(localDW->DiscreteTimeIntegrator_DSTATE - (div_s32
    (localDW->DiscreteTimeIntegrator_DSTATE, 65536) << 16));

  /* Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S2>/Constant1'
   *  Constant: '<S2>/Constant3'
   *  Product: '<S2>/Divide'
   *  Product: '<S2>/Product1'
   *  Product: '<S2>/Product2'
   */
  localDW->DiscreteTimeIntegrator_DSTATE += mul_s32_hiSR(439804651,
    div_nde_s32_floor((rtb_Saturation1_c << 16) * ((uint8_T)MOTOR_PN), 60), 10U);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
