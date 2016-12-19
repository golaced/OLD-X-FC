/*
 * File: KSW2.h
 *
 * Code generated for Simulink model 'KSW2'.
 *
 * Model version                  : 1.8
 * Simulink Coder version         : 8.7 (R2014b) 08-Sep-2014
 * C/C++ source code generated on : Fri Dec 16 10:39:35 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objective: Execution efficiency
 * Validation result: Passed (8), Warnings (4), Error (0)
 */

#ifndef RTW_HEADER_KSW2_h_
#define RTW_HEADER_KSW2_h_
#include <stddef.h>
#include <string.h>
#ifndef KSW2_COMMON_INCLUDES_
# define KSW2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* KSW2_COMMON_INCLUDES_ */

#include "KSW2_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTransferFcn_states[4];/* '<S1>/Discrete Transfer Fcn' */
} DW_KSW2_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T In1;                          /* '<Root>/In1' */
} ExtU_KSW2_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
} ExtY_KSW2_T;

/* Parameters (auto storage) */
struct P_KSW2_T_ {
  real_T DiscreteTransferFcn_NumCoef[5];/* Expression: [num_ksw2]
                                         * Referenced by: '<S1>/Discrete Transfer Fcn'
                                         */
  real_T DiscreteTransferFcn_DenCoef[5];/* Expression: [den_ksw2]
                                         * Referenced by: '<S1>/Discrete Transfer Fcn'
                                         */
  real_T DiscreteTransferFcn_InitialStat;/* Expression: 0
                                          * Referenced by: '<S1>/Discrete Transfer Fcn'
                                          */
};

/* Real-time Model Data Structure */
struct tag_RTM_KSW2_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern P_KSW2_T KSW2_P;

/* Block states (auto storage) */
extern DW_KSW2_T KSW2_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_KSW2_T KSW2_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_KSW2_T KSW2_Y;

/* Model entry point functions */
extern void KSW2_initialize(void);
extern void KSW2_step(void);
extern void KSW2_terminate(void);

/* Real-time Model object */
extern RT_MODEL_KSW2_T *const KSW2_M;

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
 * hilite_system('c_generate/KSW2')    - opens subsystem c_generate/KSW2
 * hilite_system('c_generate/KSW2/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'c_generate'
 * '<S1>'   : 'c_generate/KSW2'
 */
#endif                                 /* RTW_HEADER_KSW2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
