/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: trajectory.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 27-Feb-2022 23:16:03
 */

/* Include Files */
#include "TRAJECTORY.h"
#include "STEPPER.h"
#include <math.h>

static trajectory_state trajectorys[NUM_TRAJECTORY];

/*
 * Arguments    : const double qall[6]
 *                double Tk
 *                double call[6]
 *                double *pos
 *                double *vel
 *                double *acc
 * Return Type  : void
 */
void trajectory(double qi, double qf, double qdi, double qdf, double qddi,
                double qddf, double Tk, double *c0, double *c1, double *c2,
                double *c3, double *c4, double *c5, double *b_vmax)
{
  double b_c3_tmp;
  double c3_tmp;
  double c4_tmp;
  double c_c3_tmp;
  *c0 = qi;
  *c1 = qdi;
  *c2 = qddi / 2.0;
  c3_tmp = Tk * Tk;
  b_c3_tmp = 3.0 * c3_tmp * qddi;
  c_c3_tmp = c3_tmp * qddf;
  *c3 = -(((((20.0 * qi - 20.0 * qf) + 8.0 * Tk * qdf) + 12.0 * Tk * qdi) -
           c_c3_tmp) + b_c3_tmp) / (2.0 * rt_powd_snf(Tk, 3.0));
  c4_tmp = 14.0 * Tk * qdf;
  *c4 = (((((30.0 * qi - 30.0 * qf) + c4_tmp) + 16.0 * Tk * qdi) - 2.0 * c3_tmp *
          qddf) + b_c3_tmp) / (2.0 * rt_powd_snf(Tk, 4.0));
  b_c3_tmp = c3_tmp * qddi;
  *c5 = -(((((12.0 * qi - 12.0 * qf) + 6.0 * Tk * qdf) + 6.0 * Tk * qdi) -
           c_c3_tmp) + b_c3_tmp) / (2.0 * rt_powd_snf(Tk, 5.0));

  /*      pos = c0 + c1*T + c2*(T^2) + c3*(T^3) + c4*(T^4) + c5*(T^5); */
  /*      vel = c1 + 2*c2*T + 3*c3*(T^2) + 4*c4*(T^3) + 5*c5*(T^4); */
  /*      acc = 2*c2 + 6*c3*T + 12*c4*(T^2) + 20*c5*(T^3); */
  *b_vmax = (((((60.0 * qf - 60.0 * qi) - c4_tmp) - 14.0 * Tk * qdi) + c_c3_tmp)
             - b_c3_tmp) / 32.0 * Tk;
}

void update_circle(double x, double y, double r, double d){
	trajectory_state * trajectory = &trajectorys[0];
	Stepper_targetPosition_real(int num)
}

/*
 * File trailer for trajectory.c
 *
 * [EOF]
 */
