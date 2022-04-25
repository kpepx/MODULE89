/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: forwardKinematic.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 28-Feb-2022 02:54:27
 */

#ifndef FORWARDKINEMATIC_H
#define FORWARDKINEMATIC_H

/* Include Files */
//#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"

#define NUM_FK 2

typedef struct{
  volatile double roll;
  volatile double X;
  volatile double Y;
  volatile double Z;

}fk_state;

  /* Function Declarations */
void forwardKinematic(int num, const double q[4]);

void update_FK_real(int num);

void update_FK_Target(int num);

double get_fk_roll(int num);

double get_fk_X(int num);

double get_fk_Y(int num);

double get_fk_Z(int num);

#endif

/*
 * File trailer for forwardKinematic.h
 *
 * [EOF]
 */
