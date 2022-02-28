/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IK.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 12-Feb-2022 14:54:53
 */

/* Include Files */
#include "IK.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <stdio.h>

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
 * Arguments    : const double pos[3]
 *                double oriz
 *                double gram
 *                double qi[4]
 *                double *check
 * Return Type  : void
 */
void IK(const double pos[3], double oriz, double gram, double qi[4], double
        *check)
{
  double c2;
  double q2;
  double s2;
  qi[0] = 0.0;
  qi[1] = 0.0;
  qi[2] = 0.0;
  qi[3] = 0.0;
  c2 = (((pos[0] * pos[0] + pos[1] * pos[1]) - 61504.0) - 170362.5625) /
    204724.0;
  s2 = 1.0 - c2 * c2;
  if (s2 >= 0.0) {
    *check = 1.0;
    s2 = gram * sqrt(s2);
    q2 = rt_atan2d_snf(s2, c2);
    qi[1] = q2;
    s2 = rt_atan2d_snf(pos[1], pos[0]) - rt_atan2d_snf(412.75 * s2, 412.75 * c2
      + 248.0);
    qi[0] = s2;
    qi[2] = 249.25 - pos[2];
    qi[3] = (oriz - s2) - q2;
  } else {
    *check = 0.0;
    printf("%s\n", "Error");
    fflush(stdout);
  }
}

/*
 * File trailer for IK.c
 *
 * [EOF]
 */
