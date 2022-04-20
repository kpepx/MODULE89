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
#include "CHESS.h"
#include "IK.h"
#include <math.h>
#include "PID.h"
#include "Serial.h"

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

// num = 1 is x axis, num = 2 is y axis, num = 3 is z axis

void trajectory(int num, double qi, double qf, double qdi, double qdf, double qddi, double qddf, double Tk, double Tsample)
{
	trajectory_state * trajectory = &trajectorys[num];

	double b_c3_tmp;
	double c3_tmp;
	double c4_tmp;
	double c_c3_tmp;

	trajectory->Tk = Tk;
	trajectory->Tsam = Tsample;
	trajectory->c0 = qi;
	trajectory->c1 = qdi;
	trajectory->c2 = qddi / 2.0;
	c3_tmp = Tk * Tk;
	b_c3_tmp = 3.0 * c3_tmp * qddi;
	c_c3_tmp = c3_tmp * qddf;
	trajectory->c3 = -(((((20.0 * qi - 20.0 * qf) + 8.0 * Tk * qdf) + 12.0 * Tk * qdi) - c_c3_tmp) + b_c3_tmp) / (2.0 * pow(Tk, 3.0));
	c4_tmp = 14.0 * Tk * qdf;
	trajectory->c4 = (((((30.0 * qi - 30.0 * qf) + c4_tmp) + 16.0 * Tk * qdi) - 2.0 * c3_tmp * qddf) + b_c3_tmp) / (2.0 * pow(Tk, 4.0));
	b_c3_tmp = c3_tmp * qddi;
	trajectory->c5 = -(((((12.0 * qi - 12.0 * qf) + 6.0 * Tk * qdf) + 6.0 * Tk * qdi) - c_c3_tmp) + b_c3_tmp) / (2.0 * pow(Tk, 5.0));

	trajectory->vmax = (((((60.0 * qf - 60.0 * qi) - c4_tmp) - 14.0 * Tk * qdi) + c_c3_tmp) - b_c3_tmp) / 32.0 * Tk;
	trajectory->state = 1;
	Stepper_StartStop(num, 1);
}

void run_trajectory(int num){
	trajectory_state * trajectory = &trajectorys[num];
	if(trajectory->state){
		if(Stepper_status(num) != 0x80){
			trajectory->T += trajectory->Tsam;
			trajectory->pos = trajectory->c0 + trajectory->c1*trajectory->T + trajectory->c2*pow(trajectory->T, 2.00) + trajectory->c3*pow(trajectory->T, 3.00) + trajectory->c4*pow(trajectory->T, 4.00) + trajectory->c5*pow(trajectory->T, 5.00);
			trajectory->vel = trajectory->c1 + 2*trajectory->c2*trajectory->T + 3*trajectory->c3*pow(trajectory->T, 2.00) + 4*trajectory->c4*pow(trajectory->T, 3.00) + 5*trajectory->c5*pow(trajectory->T, 4.00);
			trajectory->acc = 2*trajectory->c2 + 6*trajectory->c3*trajectory->T + 12*trajectory->c4*pow(trajectory->T, 2.00) + 20*trajectory->c5*pow(trajectory->T, 3.00);
		}

	}
	if(trajectory->T >= trajectory->Tk){
		trajectory->T = 0;
		Stepper_StartStop(num, 0);
	}
}

void reset_trajectory(int num){
	trajectory_state * trajectory = &trajectorys[num];
	trajectory->T = 0;
	trajectory->state = 0;
}

/*
 * File trailer for trajectory.c
 *
 * [EOF]
 */
