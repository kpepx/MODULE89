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
//#include "rt_nonfinite.h"
//#include "rt_defines.h"
//#include "rt_nonfinite.h"
#include <math.h>
#include <stdio.h>

/*
 * Arguments    : const double pos[3]
 *                double oriz
 *                double gram
 *                double qi[4]
 *                double *check
 * Return Type  : void
 */

static ik_state iks[NUM_IK];

void IK(double pos[3], double oriz, double gram)
{
	ik_state * ik = &iks[0];
  double c2;
  double q2;
  double s2;
  c2 = (((pos[0] * pos[0] + pos[1] * pos[1]) - 61504.0) - 170362.5625) /
    204724.0;
  s2 = 1.0 - c2 * c2;
  if (s2 >= 0.0) {
    ik->check = 1;
    s2 = gram * sqrt(s2);
    q2 = atan2(s2, c2);

    ik->q2 = q2;
    s2 = atan2(pos[1], pos[0]) - atan2(412.75 * s2, 412.75 * c2
      + 248.0);
    ik->q1 = s2;
    ik->q3 = 249.25 - pos[2];
    ik->q4 = (oriz - s2) - q2;
  } else {
    ik->check = 0;
  }
}

double get_ik_q1(){
	ik_state * ik = &iks[0];
	return ik->q1;
}

double get_ik_q2(){
	ik_state * ik = &iks[0];
	return ik->q2;
}

double get_ik_q3(){
	ik_state * ik = &iks[0];
	return ik->q3;
}

double get_ik_q4(){
	ik_state * ik = &iks[0];
	return ik->q4;
}

/*
 * File trailer for IK.c
 *
 * [EOF]
 */
