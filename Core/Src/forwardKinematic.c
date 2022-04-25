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
#include "STEPPER.h"
#include <math.h>

static fk_state fks[NUM_FK];

void forwardKinematic(int num, const double q[4])
{
	fk_state * fk = &fks[num];
	double Rota_idx_1;
	double Rota_tmp;
	double Rota_tmp_tmp_tmp;
	double eulShaped_idx_2;
	Rota_tmp_tmp_tmp = q[0] + q[1];
	Rota_idx_1 = Rota_tmp_tmp_tmp + q[3];
	Rota_tmp = cos(Rota_idx_1);
	Rota_idx_1 = sin(Rota_idx_1);
	eulShaped_idx_2 = atan2(Rota_idx_1, Rota_tmp);
	if (sqrt(Rota_tmp * Rota_tmp + Rota_idx_1 * Rota_idx_1) <
			2.2204460492503131E-15) {
		eulShaped_idx_2 = 0.0;
	}
	fk->roll = eulShaped_idx_2; //radian
	fk->X = 412.97 * cos(Rota_tmp_tmp_tmp) + 248.0 * cos(q[0]); //mm
	fk->Y = 412.97 * sin(Rota_tmp_tmp_tmp) + 248.0 * sin(q[0]); //mm
	fk->Z = 217.04 - q[2]; //mm
}

void update_FK_real(int num){
	double qi_all[4] = {to_radian((double)Stepper_currentPosition_real(1)), to_radian((double)Stepper_currentPosition_real(2)), (double)Stepper_currentPosition_real(3), to_radian((double)0.0)};
	forwardKinematic(num, qi_all);
}

void update_FK_Target(int num){
	double qi_all[4] = {to_radian((double)Stepper_targetPosition_real(1)), to_radian((double)Stepper_targetPosition_real(2)), (double)Stepper_targetPosition_real(3), to_radian((double)0.0)};
	forwardKinematic(num, qi_all);
}

double get_fk_roll(int num){
	fk_state * fk = &fks[num];
	return fk->roll;
}

double get_fk_X(int num){
	fk_state * fk = &fks[num];
	return fk->X;
}

double get_fk_Y(int num){
	fk_state * fk = &fks[num];
	return fk->Y;
}

double get_fk_Z(int num){
	fk_state * fk = &fks[num];
	return fk->Z;
}

/*
 * File trailer for forwardKinematic.c
 *
 * [EOF]
 */
