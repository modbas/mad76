/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Motion_types.h
 *
 * Code generated for Simulink model 'Motion'.
 *
 * Model version                  : 11.541
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Wed Feb 18 12:57:11 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-A (64-bit)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef Motion_types_h_
#define Motion_types_h_
#include "rtwtypes.h"
#include "TimeMsg.h"
#include "CheckpointMsg.h"
#include "CheckpointSequenceMsg.h"
#ifndef DEFINED_TYPEDEF_FOR_CAROBS_
#define DEFINED_TYPEDEF_FOR_CAROBS_

typedef struct {
  real32_T s[2];
  real32_T psi;
  real32_T beta;
  real32_T v;
  real32_T cxe;
  real32_T cx;
  real32_T cxd;
  real32_T cs[2];
  real32_T cey;
  real32_T cepsi;
  real32_T ckappa;
  real32_T rey;
  real32_T ley;
  real32_T prob;
} CAROBS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CARINPUTS_
#define DEFINED_TYPEDEF_FOR_CARINPUTS_

typedef struct {
  uint8_T cmd;
  real32_T pedals;
  real32_T steering;
  CHECKPOINTSEQUENCEMSG cpseq;
} CARINPUTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CTRLREFERENCE_
#define DEFINED_TYPEDEF_FOR_CTRLREFERENCE_

typedef struct {
  real32_T s[2];
  real32_T psi;
  real32_T v;
} CTRLREFERENCE;

#endif

/* Custom Type definition for MATLAB Function: '<S6>/PolicyWrapper' */
#ifndef struct_tag_HZeJCHSLzOYxEF59RDVu9
#define struct_tag_HZeJCHSLzOYxEF59RDVu9

struct tag_HZeJCHSLzOYxEF59RDVu9
{
  real32_T f1[2];
};

#endif                                 /* struct_tag_HZeJCHSLzOYxEF59RDVu9 */

#ifndef typedef_cell_wrap_11_Motion_T
#define typedef_cell_wrap_11_Motion_T

typedef struct tag_HZeJCHSLzOYxEF59RDVu9 cell_wrap_11_Motion_T;

#endif                                 /* typedef_cell_wrap_11_Motion_T */

#ifndef struct_tag_4ECu3HFT3NowpyqvgWxJAC
#define struct_tag_4ECu3HFT3NowpyqvgWxJAC

struct tag_4ECu3HFT3NowpyqvgWxJAC
{
  boolean_T f1[2];
};

#endif                                 /* struct_tag_4ECu3HFT3NowpyqvgWxJAC */

#ifndef typedef_cell_wrap_12_Motion_T
#define typedef_cell_wrap_12_Motion_T

typedef struct tag_4ECu3HFT3NowpyqvgWxJAC cell_wrap_12_Motion_T;

#endif                                 /* typedef_cell_wrap_12_Motion_T */

#ifndef struct_tag_v04emYtH0gA9oC4XEZTQ9G
#define struct_tag_v04emYtH0gA9oC4XEZTQ9G

struct tag_v04emYtH0gA9oC4XEZTQ9G
{
  real32_T f1[97];
};

#endif                                 /* struct_tag_v04emYtH0gA9oC4XEZTQ9G */

#ifndef typedef_cell_wrap_15_Motion_T
#define typedef_cell_wrap_15_Motion_T

typedef struct tag_v04emYtH0gA9oC4XEZTQ9G cell_wrap_15_Motion_T;

#endif                                 /* typedef_cell_wrap_15_Motion_T */

#ifndef struct_tag_xviDWIkgrKK5ptWTEU1YkF
#define struct_tag_xviDWIkgrKK5ptWTEU1YkF

struct tag_xviDWIkgrKK5ptWTEU1YkF
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 /* struct_tag_xviDWIkgrKK5ptWTEU1YkF */

#ifndef typedef_rl_codegen_model_DLNetworkMod_T
#define typedef_rl_codegen_model_DLNetworkMod_T

typedef struct tag_xviDWIkgrKK5ptWTEU1YkF rl_codegen_model_DLNetworkMod_T;

#endif                             /* typedef_rl_codegen_model_DLNetworkMod_T */

#ifndef struct_tag_U9bHemCUpAFrGGNyvUKfZ
#define struct_tag_U9bHemCUpAFrGGNyvUKfZ

struct tag_U9bHemCUpAFrGGNyvUKfZ
{
  cell_wrap_11_Motion_T Scale_;
  cell_wrap_11_Motion_T Bias_;
  cell_wrap_12_Motion_T UpperBoundedIdx_;
  cell_wrap_12_Motion_T LowerBoundedIdx_;
};

#endif                                 /* struct_tag_U9bHemCUpAFrGGNyvUKfZ */

#ifndef typedef_c_rl_codegen_policy_internal__T
#define typedef_c_rl_codegen_policy_internal__T

typedef struct tag_U9bHemCUpAFrGGNyvUKfZ c_rl_codegen_policy_internal__T;

#endif                             /* typedef_c_rl_codegen_policy_internal__T */

#ifndef struct_tag_OvFQNXfs6VrJsU8wwkq9cF
#define struct_tag_OvFQNXfs6VrJsU8wwkq9cF

struct tag_OvFQNXfs6VrJsU8wwkq9cF
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  rl_codegen_model_DLNetworkMod_T *Model_;
  boolean_T UseMaxLikelihoodAction_;
  c_rl_codegen_policy_internal__T ActionBounder_;
};

#endif                                 /* struct_tag_OvFQNXfs6VrJsU8wwkq9cF */

#ifndef typedef_c_rl_codegen_policy_rlContinu_T
#define typedef_c_rl_codegen_policy_rlContinu_T

typedef struct tag_OvFQNXfs6VrJsU8wwkq9cF c_rl_codegen_policy_rlContinu_T;

#endif                             /* typedef_c_rl_codegen_policy_rlContinu_T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_Motion_T RT_MODEL_Motion_T;

#endif                                 /* Motion_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
