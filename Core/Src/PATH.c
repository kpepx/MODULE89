// File name: PATH.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "PATH.h"
#include "TRAJECTORY.h"
#include "CHESS.h"
#include "STEPPER.h"
#include "forwardKinematic.h"
#include "IK.h"
#include "Serial.h"
#include <math.h>

uint32_t time2 = 0;
int z_offset = 100.00;
int y_offset = 0.00;
int x_offset = 0.00;

static path_state paths[NUM_PATH];

void path(int32_t row, int32_t column, int32_t type){
	path_state * path = &paths[0];
	path->row = row;
	path->column = column;
	update_FK_Target(1);
	double t = find_time(get_fk_X(1), get_fk_Y(1), 398.00 + x_offset, -6.00 + y_offset);
	trajectory(1, get_fk_X(1), 398.00 + x_offset, 0, 0, 0, 0, t, 0.001);
	trajectory(2, get_fk_Y(1), -6.00 + y_offset, 0, 0, 0, 0, t, 0.001);
	trajectory(3, get_fk_Z(1), 216.00, 0, 0, 0, 0, t, 0.001);
	path->check = 1;
	path->type = type;
	if(path->type == PICK){
		Servo_gripperChess(2, 0);
	}
	else if(path->type == PLACE){
		Servo_gripperChess(2, 60*100.00);
	}
}

double find_time(int32_t xi, int32_t yi, int32_t xf, int32_t yf){
	return sqrt(pow(xf - xi, 2.00) + pow(yf - yi, 2.00))/3.00;
}

void update_path(){
	path_state * path = &paths[0];

	if(path->check != 0){
		run_trajectory(1);
		run_trajectory(2);
		run_trajectory(3);
		double xyz[3] = {get_pos(1), get_pos(2), get_pos(3)};
		IK(0, xyz, 0, -1);
		Stepper_SetTraget(1, to_degree(get_ik_q1(0)));
		Stepper_SetTraget(2, to_degree(get_ik_q2(0)));
		Stepper_SetTraget(3, get_ik_q3(0));
//		if((HAL_GetTick()-time2)>1000){
//			time2 = HAL_GetTick();
//			Servo_tragetPos(2, to_degree(get_ik_q4(0))*100.00);
//		}
	}
	switch(path->check){
	case 1:
		if(get_check_trajectory(1) == 1){
			reset_trajectory(1);
			reset_trajectory(2);
			reset_trajectory(3);
			updateChess(path->row, path->column);
			update_FK_Target(1);
			double t = find_time(get_fk_X(1), get_fk_Y(1), get_fk_X(1) + get_x_chess(), get_fk_Y(1) + get_y_chess());
//			path->x = get_fk_X(1) + get_x_chess();
//			path->y = get_fk_Y(1) + get_y_chess();
//			double xy[3] = {path->x, path->y, 0};
//			IK(1, xy, 0 ,-1);
			trajectory(1, get_fk_X(1), get_fk_X(1) + get_x_chess(), 0, 0, 0, 0, t, 0.001);
			trajectory(2, get_fk_Y(1), get_fk_Y(1) + get_y_chess(), 0, 0, 0, 0, t, 0.001);
			trajectory(3, 216.00, 216.00, 0, 0, 0, 0, t, 0.001);
			path->check = 2;
		}
		break;
	case 2:
//		Stepper_SetTraget(1, to_degree(get_ik_q1(0)));
//		Stepper_SetTraget(2, to_degree(get_ik_q2(0)));
//		Stepper_SetTraget(3, get_ik_q3(0));
//		Servo_tragetPos(2, to_degree(get_ik_q4(0)));
		if(get_check_trajectory(1) == 1){
			reset_trajectory(1);
			reset_trajectory(2);
			reset_trajectory(3);
			update_FK_Target(1);
			trajectory(3, 216.00, 216.00 - z_offset, 0, 0, 0, 0, 20, 0.001); //z down
			path->check = 0;
		}
		break;
	case 3:
//		Stepper_SetTraget(3, get_ik_q3(0));
		if(get_check_trajectory(3) == 1){
			reset_trajectory(1);
			reset_trajectory(2);
			reset_trajectory(3);
			if(path->type == PICK){
				Servo_gripperChess(2, 60*100.00);
			}
			else if(path->type == PLACE){
				Servo_gripperChess(2, 0);
			}
			HAL_Delay(500);
			update_FK_Target(1);
			trajectory(3, 216.00 - z_offset, 216.00, 0, 0, 0, 0, 17, 0.001); //z up
			path->check = 4;
		}
		break;
	case 4:
//		Stepper_SetTraget(3, get_ik_q3(0));
		if(get_check_trajectory(3) == 1){
			reset_trajectory(1);
			reset_trajectory(2);
			reset_trajectory(3);
			update_FK_Target(1);
			double t = find_time(get_fk_X(1), get_fk_Y(1), 80, -160.00);
			trajectory(1, get_fk_X(1), 80.00, 0, 0, 0, 0, t, 0.001);
			trajectory(2, get_fk_Y(1), -160.00, 0, 0, 0, 0, t, 0.001);
			path->check = 5;
		}
		break;
	case 5:
//		Stepper_SetTraget(1, to_degree(get_ik_q1(0)));
//		Stepper_SetTraget(2, to_degree(get_ik_q2(0)));
//		Stepper_SetTraget(3, get_ik_q3(0));
//		Servo_tragetPos(2, to_degree(get_ik_q4(0)));
		if(get_check_trajectory(1) == 1){
			reset_trajectory(1);
			reset_trajectory(2);
			reset_trajectory(3);
			path->check = 0;
			Feedback_Complete(1, 99);
		}
		break;
	}


}
