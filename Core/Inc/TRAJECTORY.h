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

#define NUM_TRAJECTORY 1

typedef struct{
	volatile double x_circle;
	volatile double y_circle;
	volatile double x_circle_target;
	volatile double y_circle_target;
	volatile double r_circle;
	volatile double d_circle; //ref horizontal
	volatile double w_circle;
	volatile double t_circle;
	volatile double t_sum_circle;
	volatile double d_robot;


}trajectory_state;

  /* Function Declarations */
extern void trajectory(double qi, double qf, double qdi, double qdf, double qddi,
          double qddf, double Tk, double *c0, double *c1, double *c2,
          double *c3, double *c4, double *c5, double *b_vmax);

void update_circle(int32_t row, int32_t column, int32_t w, int32_t t);


#endif /* __TRAJECTORY_H */

/*
 * File trailer for trajectory.h
 *
 * [EOF]
 */
