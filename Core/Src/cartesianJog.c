/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: cartesianJog.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 27-Feb-2022 18:10:00
 */

/* Include Files */
#include "cartesianJog.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double qi[4]
 *                const double deltak[4]
 *                double qf[4]
 * Return Type  : void
 */

static cartesian_state cartesians[NUM_CARTESIAN];

void cartesianJog(const double qi[4], const double deltak[4])
{
	cartesian_state * cartesian = &cartesians[0];
	double b_qf_tmp;
	double c_qf_tmp;
	double d_qf_tmp;
	double qf_tmp;
	double qf_tmp_tmp;
	qf_tmp = sin(qi[1]);
	qf_tmp_tmp = qi[0] + qi[1];
	b_qf_tmp = cos(qf_tmp_tmp);
	qf_tmp_tmp = sin(qf_tmp_tmp);
	c_qf_tmp = cos(qi[0]);
	d_qf_tmp = sin(qi[0]);

	cartesian->q1 = ((deltak[1] * b_qf_tmp + deltak[2] * qf_tmp_tmp) + 248.0 * qi[0] * qf_tmp) / (248.0 * qf_tmp);
	cartesian->q2 = (qi[1] - deltak[1] * (1651.0 * b_qf_tmp / 4.0 + 248.0 * c_qf_tmp) / (102362.0 * qf_tmp)) - deltak[2] * (1651.0 * qf_tmp_tmp / 4.0 + 248.0 * d_qf_tmp) / (102362.0 * sin(qi[1]));
	cartesian->q3 = qi[2] - deltak[3];
	cartesian->q4 = (((4.0 * deltak[1] * c_qf_tmp + 1651.0 * deltak[0] * qf_tmp) + 4.0 * deltak[2] * d_qf_tmp) + 1651.0 * qi[3] * qf_tmp) / (1651.0 * qf_tmp);
}

double get_cartesian_q1(){
	cartesian_state * cartesian = &cartesians[0];
	return cartesian->q1;
}

double get_cartesian_q2(){
	cartesian_state * cartesian = &cartesians[0];
	return cartesian->q2;
}

double get_cartesian_q3(){
	cartesian_state * cartesian = &cartesians[0];
	return cartesian->q3;
}

double get_cartesian_q4(){
	cartesian_state * cartesian = &cartesians[0];
	return cartesian->q4;
}

/*
 * File trailer for cartesianJog.c
 *
 * [EOF]
 */
