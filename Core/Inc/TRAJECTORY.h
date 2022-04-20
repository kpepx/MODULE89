/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: trajectory.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 27-Feb-2022 23:16:03
 */

#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#include "stm32h7xx_hal.h"

#define NUM_TRAJECTORY 4

typedef struct{
	volatile double c0, c1, c2, c3, c4, c5;
	volatile double vmax;
	volatile double Tk, Tsam, T;
	volatile int state;
	volatile double pos, vel, acc;

}trajectory_state;

  /* Function Declarations */
extern void trajectory(int num, double qi, double qf, double qdi, double qdf, double qddi, double qddf, double Tk, double Tsample);

void run_trajectory(int num);

void reset_trajectory(int num);


#endif /* __TRAJECTORY_H */

/*
 * File trailer for trajectory.h
 *
 * [EOF]
 */
