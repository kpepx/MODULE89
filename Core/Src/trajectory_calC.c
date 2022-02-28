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
#include "trajectory.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double d;
  double d1;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : const double qall[6]
 *                double Tk
 *                double call[6]
 *                double *pos
 *                double *vel
 *                double *acc
 * Return Type  : void
 */
void trajectory(const double qall[6], double Tk, double call[6], double *pos,
                double *vel, double *acc)
{
  double b_call_tmp;
  double c_call_tmp;
  double call_tmp;
  double d_call_tmp;
  double e_call_tmp;
  call[0] = qall[0];
  call[1] = qall[2];
  call[2] = 2.0 * qall[4];
  call_tmp = Tk * Tk;
  b_call_tmp = 3.0 * call_tmp * qall[4];
  c_call_tmp = call_tmp * qall[5];
  d_call_tmp = rt_powd_snf(Tk, 3.0);
  call[3] = -(((((20.0 * qall[0] - 20.0 * qall[1]) + 8.0 * Tk * qall[3]) + 12.0 *
                Tk * qall[2]) - c_call_tmp) + b_call_tmp) / (2.0 * d_call_tmp);
  e_call_tmp = rt_powd_snf(Tk, 4.0);
  call[4] = (((((30.0 * qall[0] - 30.0 * qall[1]) + 14.0 * Tk * qall[3]) + 16.0 *
               Tk * qall[2]) - 2.0 * call_tmp * qall[5]) + b_call_tmp) / (2.0 *
    e_call_tmp);
  b_call_tmp = rt_powd_snf(Tk, 5.0);
  call[5] = -(((((12.0 * qall[0] - 12.0 * qall[1]) + 6.0 * Tk * qall[3]) + 6.0 *
                Tk * qall[2]) - c_call_tmp) + call_tmp * qall[4]) / (2.0 *
    b_call_tmp);
  *pos = ((((qall[0] + qall[2] * Tk) + call[2] * call_tmp) + call[3] *
           d_call_tmp) + call[4] * e_call_tmp) + call[5] * b_call_tmp;
  *vel = (((qall[2] + 2.0 * call[2] * Tk) + 3.0 * call[3] * call_tmp) + 4.0 *
          call[4] * d_call_tmp) + 5.0 * call[5] * e_call_tmp;
  *acc = ((2.0 * call[2] + 6.0 * call[3] * Tk) + 12.0 * call[4] * call_tmp) +
    20.0 * call[5] * d_call_tmp;
}

/*
 * File trailer for trajectory.c
 *
 * [EOF]
 */
