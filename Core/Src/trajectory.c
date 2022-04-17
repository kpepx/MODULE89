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

void update_circle(int32_t row, int32_t column, int32_t w, int32_t t){
	trajectory_state * trajectory = &trajectorys[0];

	updateChess(row, column);

	trajectory->x_circle = 200.00;
	trajectory->y_circle = 200.00;
	trajectory->r_circle = get_radius_circle();
	trajectory->w_circle = w/100.00;
	trajectory->t_circle = t/10000.00;
	trajectory->d_circle = get_degree_chess();
}

void run_trajectory_circle(){
	trajectory_state * trajectory = &trajectorys[0];

//	trajectory->d_robot = find_degree(trajectory->x_circle_target, trajectory->y_circle_target); //real robot position
//	double degree_field = get_real_degree_chess(); //real chess position

	double real_x_chess = trajectory->x_circle + get_x_chess();
	double real_y_chess = trajectory->y_circle + get_y_chess();

	double dx = real_x_chess - trajectory->x_circle_target;
	double dy = real_y_chess - trajectory->y_circle_target;
	double euclidean_distance = sqrt(pow(dx, 2) + pow(dy, 2));
	double degree = asin(euclidean_distance/(2*trajectory->r_circle))*2;

	trajectory->t_sum_circle += trajectory->t_circle*degree;

	double d = trajectory->d_circle * (M_PI/180.00);
	trajectory->x_circle_target = trajectory->x_circle + trajectory->r_circle*cos(trajectory->w_circle*trajectory->t_sum_circle + d);
	trajectory->y_circle_target = trajectory->y_circle + trajectory->r_circle*sin(trajectory->w_circle*trajectory->t_sum_circle + d);

	double xyz[3] = {trajectory->x_circle_target, trajectory->y_circle_target, 0};
	IK(xyz, 0, 1);

	//run actuator
	Stepper_SetTraget(1, to_degree(get_ik_q1()));
	Stepper_SetTraget(2, to_degree(get_ik_q2()));
	Stepper_SetTraget(3, get_ik_q3());
	Servo_tragetPos(2, to_degree(get_ik_q4()));
}

/*
 * File trailer for trajectory.c
 *
 * [EOF]
 */
