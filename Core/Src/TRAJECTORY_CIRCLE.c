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
#include "TRAJECTORY_CIRCLE.h"
#include "STEPPER.h"
#include "CHESS.h"
#include "IK.h"
#include <math.h>
#include "PID.h"
#include "Serial.h"

volatile int on_robot = 0;

static trajectory_circle_state trajectory_circles[NUM_TRAJECTORY_CIRCLE];

void update_circle(int32_t row, int32_t column, int32_t w, int32_t t){
	trajectory_circle_state * trajectory_circle = &trajectory_circles[0];

	updateChess(row, column);

	trajectory_circle->x_circle = 200.00;
	trajectory_circle->y_circle = 200.00;
	trajectory_circle->r_circle = get_radius_circle();
	trajectory_circle->w_circle = w/100.00;
	trajectory_circle->t_circle = t/10000.00;
	trajectory_circle->d_circle = get_degree_chess();
	on_robot = 1;
}

void run_trajectory_circle(){
	trajectory_circle_state * trajectory_circle = &trajectory_circles[0];

	if(on_robot){
		double real_x_chess = trajectory_circle->x_circle + get_x_chess();
		double real_y_chess = trajectory_circle->y_circle + get_y_chess();

		double dx = real_x_chess - trajectory_circle->x_circle_target;
		double dy = real_y_chess - trajectory_circle->y_circle_target;
		double euclidean_distance = sqrt(pow(dx, 2) + pow(dy, 2));
		double degree = asin(euclidean_distance/(2*trajectory_circle->r_circle))*2; //radius unit

		if(degree < 0.005){
			trajectory_circle->z_robot = 100;
		}

		trajectory_circle->t_sum_circle += trajectory_circle->t_circle*degree;

		double d = trajectory_circle->d_circle * (M_PI/180.00);
		trajectory_circle->x_circle_target = trajectory_circle->x_circle + trajectory_circle->r_circle*cos(trajectory_circle->w_circle*trajectory_circle->t_sum_circle + d);
		trajectory_circle->y_circle_target = trajectory_circle->y_circle + trajectory_circle->r_circle*sin(trajectory_circle->w_circle*trajectory_circle->t_sum_circle + d);

		double xyz[3] = {trajectory_circle->x_circle_target, trajectory_circle->y_circle_target, trajectory_circle->z_robot};
		IK(0, xyz, 0, 1);

		//run actuator
		Stepper_SetTraget(1, to_degree(get_ik_q1(0)));
		Stepper_SetTraget(2, to_degree(get_ik_q2(0)));
		Stepper_SetTraget(3, get_ik_q3(0));
		Servo_tragetPos(2, to_degree(get_ik_q4(0))*100.00);
	}
}

/*
 * File trailer for trajectory.c
 *
 * [EOF]
 */
