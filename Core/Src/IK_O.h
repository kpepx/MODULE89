/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IK.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 12-Feb-2022 14:54:53
 */

#ifndef IK_H
#define IK_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void IK(const double pos[3], double oriz, double gram, double qi[4],
                 double *check);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for IK.h
 *
 * [EOF]
 */
