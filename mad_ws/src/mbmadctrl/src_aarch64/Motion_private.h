/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Motion_private.h
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

#ifndef Motion_private_h_
#define Motion_private_h_
#include "rtwtypes.h"
#include "Motion_types.h"
#include "Motion.h"

extern void microKernel13792962022813235949(int32_T K, const real32_T *A,
  int32_T LDA, const real32_T *B, real32_T *C);
extern void microKernel12334948770678980866(int32_T K, const real32_T *A,
  int32_T LDA, const real32_T *B, real32_T *C);
extern void microKernel3050070057585809138(int32_T K, const real32_T *A, int32_T
  LDA, const real32_T *B, real32_T *C);
extern void macroKernel10385924302351677068(int32_T M, int32_T K, int32_T N,
  const real32_T *A, int32_T LDA, const real32_T *B, int32_T LDB, real32_T *C,
  int32_T LDC);
extern void matrixMultiply10385924302351677068(int32_T M, int32_T K, int32_T N,
  int32_T blockSizeM, int32_T blockSizeK, int32_T blockSizeN, const real32_T *A,
  const real32_T *B, real32_T *C);
extern void macroKernel6108398620961628910(int32_T M, int32_T K, int32_T N,
  const real32_T *A, int32_T LDA, const real32_T *B, int32_T LDB, real32_T *C,
  int32_T LDC);
extern void matrixMultiply6108398620961628910(int32_T M, int32_T K, int32_T N,
  int32_T blockSizeM, int32_T blockSizeK, int32_T blockSizeN, const real32_T *A,
  const real32_T *B, real32_T *C);
extern int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);
extern int32_T div_s32_floor(int32_T numerator, int32_T denominator);

#endif                                 /* Motion_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
