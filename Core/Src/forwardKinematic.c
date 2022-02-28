/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: forwardKinematic.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 28-Feb-2022 02:54:27
 */

/* Include Files */
#include "forwardKinematic.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : const double q[4]
 *                double klee[4]
 * Return Type  : void
 */
void forwardKinematic(const double q[4], double klee[4])
{
  double Rota_idx_1;
  double Rota_tmp;
  double Rota_tmp_tmp_tmp;
  double eulShaped_idx_2;
  Rota_tmp_tmp_tmp = q[0] + q[1];
  Rota_idx_1 = Rota_tmp_tmp_tmp + q[3];
  Rota_tmp = cos(Rota_idx_1);
  Rota_idx_1 = sin(Rota_idx_1);
  eulShaped_idx_2 = rt_atan2d_snf(Rota_idx_1, Rota_tmp);
  if (sqrt(Rota_tmp * Rota_tmp + Rota_idx_1 * Rota_idx_1) <
      2.2204460492503131E-15) {
    eulShaped_idx_2 = 0.0;
  }

  klee[0] = eulShaped_idx_2;
  klee[1] = 412.75 * cos(Rota_tmp_tmp_tmp) + 248.0 * cos(q[0]);
  klee[2] = 412.75 * sin(Rota_tmp_tmp_tmp) + 248.0 * sin(q[0]);
  klee[3] = 249.25 - q[2];
}

/*
 * File trailer for forwardKinematic.c
 *
 * [EOF]
 */
