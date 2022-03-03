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
//#include "rt_nonfinite.h"
//#include "rt_defines.h"
//#include "rt_nonfinite.h"
#include <math.h>

static fk_state fks[NUM_FK];

void forwardKinematic(const double q[4])
{
	fk_state * fk = &fks[0];
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

  fk->roll = eulShaped_idx_2; //degree
  fk->X = 412.75 * cos(Rota_tmp_tmp_tmp) + 248.0 * cos(q[0]); //mm
  fk->Y = 412.75 * sin(Rota_tmp_tmp_tmp) + 248.0 * sin(q[0]); //mm
  fk->Z = 249.25 - q[2]; //mm
}

double get_fk_roll(){
	fk_state * fk = &fks[0];
	return fk->roll;
}

double get_fk_X(){
	fk_state * fk = &fks[0];
	return fk->X;
}

double get_fk_Y(){
	fk_state * fk = &fks[0];
	return fk->Y;
}

double get_fk_Z(){
	fk_state * fk = &fks[0];
	return fk->Z;
}

/*
 * File trailer for forwardKinematic.c
 *
 * [EOF]
 */
