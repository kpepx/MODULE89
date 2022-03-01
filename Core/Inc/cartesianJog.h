/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: cartesianJog.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 27-Feb-2022 18:10:00
 */

#ifndef __CARTESIANJOG_H
#define __CARTESIANJOG_H

/* Include Files */
//#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"

#define NUM_CARTESIAN 1

typedef struct{
  volatile double q1;
  volatile double q2;
  volatile double q3;
  volatile double q4;

}cartesian_state;

void cartesianJog(double qi[4], double deltak[4]);

double get_cartesian_q1();

double get_cartesian_q2();

double get_cartesian_q3();

double get_cartesian_q4();

#endif /* __CARTESIANJOG_H */
/*
 * File trailer for cartesianJog.h
 *
 * [EOF]
 */
