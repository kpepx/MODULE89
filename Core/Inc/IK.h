/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IK.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 12-Feb-2022 14:54:53
 */

#ifndef __IK_H
#define __IK_H

/* Include Files */
//#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"

#define NUM_IK 1

typedef struct{
  volatile double q1;
  volatile double q2;
  volatile double q3;
  volatile double q4;

}ik_state;

/* Function Declarations */
void IK(const double pos[3], double oriz, double gram, double qi[4], double *check);


#endif /* __IK_H */

/*
 * File trailer for IK.h
 *
 * [EOF]
 */
