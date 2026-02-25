/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CheckpointSequenceMsg.h
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

#ifndef CheckpointSequenceMsg_h_
#define CheckpointSequenceMsg_h_
#include "CheckpointMsg.h"
#include "rtwtypes.h"

typedef struct {
  CHECKPOINTMSG cpseq[23];
  boolean_T health;
} CHECKPOINTSEQUENCEMSG;

#endif                                 /* CheckpointSequenceMsg_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
