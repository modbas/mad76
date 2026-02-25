/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Motion.h
 *
 * Code generated for Simulink model 'Motion'.
 *
 * Model version                  : 11.541
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Wed Feb 18 12:52:37 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef Motion_h_
#define Motion_h_
#ifndef Motion_COMMON_INCLUDES_
#define Motion_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* Motion_COMMON_INCLUDES_ */

#include "Motion_types.h"
#include "CheckpointSequenceMsg.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  c_rl_codegen_policy_rlContinu_T policy;/* '<S6>/PolicyWrapper' */
  rl_codegen_model_DLNetworkMod_T gobj_1;/* '<S6>/PolicyWrapper' */
  real32_T UnitDelay3_DSTATE[2];       /* '<S7>/Unit Delay3' */
  real32_T UnitDelay2_DSTATE[2];       /* '<S7>/Unit Delay2' */
  real32_T UnitDelay1_DSTATE[2];       /* '<S7>/Unit Delay1' */
  uint32_T state[625];                 /* '<S6>/PolicyWrapper' */
  boolean_T policy_not_empty;          /* '<S6>/PolicyWrapper' */
} DW_Motion_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T carid;                        /* '<Root>/carid' */
  CAROBS carobslist[4];                /* '<Root>/carobs' */
} ExtU_Motion_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  CARINPUTS inputs;                    /* '<Root>/inputs' */
  CTRLREFERENCE ctrlref;               /* '<Root>/ctrlref' */
  boolean_T leadcrash;                 /* '<Root>/leadcrash' */
  real32_T leadv;                      /* '<Root>/leadv' */
} ExtY_Motion_T;

/* Real-time Model Data Structure */
struct tag_RTM_Motion_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_Motion_T Motion_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Motion_T Motion_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Motion_T Motion_Y;

/* Model entry point functions */
extern void Motion_initialize(void);
extern void Motion_step(void);
extern void Motion_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Motion_T *const Motion_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S4>/Gain2' : Unused code path elimination
 * Block '<S4>/Logical Operator' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('s48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC')    - opens subsystem s48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC
 * hilite_system('s48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 's48_sig_rl_race_pylons/Car0'
 * '<S1>'   : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC'
 * '<S2>'   : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control'
 * '<S3>'   : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Agent'
 * '<S4>'   : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Obs'
 * '<S5>'   : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Agent/Policy'
 * '<S6>'   : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Agent/Policy/Policy'
 * '<S7>'   : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Agent/Policy/delay action'
 * '<S8>'   : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Agent/Policy/Policy/PolicyWrapper'
 * '<S9>'   : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Obs/CrashDetection'
 * '<S10>'  : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Obs/Obs Leading Car1'
 * '<S11>'  : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Obs/Pylons'
 * '<S12>'  : 's48_sig_rl_race_pylons/Car0/Motion Control Pi Act Delay SAC/Control/Obs/Obs Leading Car1/Lead Localisation'
 */
#endif                                 /* Motion_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
