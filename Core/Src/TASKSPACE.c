// File name: TASKSPACE.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "TASKSPACE.h"
#include "STEPPER.h"
#include "cartesianJog.h"
#include "forwardKinematic.h"
#include "IK.h"
#include <math.h>

static taskspace_state taskspaces[NUM_TASKSPACE];

void updateXYZ(int32_t x, int32_t y, int32_t z){
//	taskspace_state * taskspace = &taskspaces[0];
}

double to_degree(double value){
	return value * 180.0 / M_PI;
}

double to_radian(double value){
	return value * M_PI / 180.0;
}

void updateJoint(int32_t roll, int32_t x, int32_t y, int32_t z){
	taskspace_state * taskspace = &taskspaces[0];
	taskspace->qi1 = to_radian((double)Stepper_currentPosition_real(1));
	taskspace->qi2 = to_radian((double)Stepper_currentPosition_real(2));
	taskspace->qi3 = (double)Stepper_currentPosition_real(3);
	taskspace->qi4 = to_radian((double)0.0);

	double qi_all[4] = {taskspace->qi1, taskspace->qi2, taskspace->qi3, taskspace->qi4};
//	double qi_all[4] = {0.0, M_PI / 2.0, 0.0, 0.0};
//	double q_0[4] = {0.0, 0.0, 0.0, 0.0};
	forwardKinematic(qi_all);

	taskspace->d1 = (double)(roll/100.00);
	taskspace->d2 = (double)(x/100.00);
	taskspace->d3 = (double)(y/100.00);
	taskspace->d4 = (double)(z/100.00);
	double delta[4] = {taskspace->d1, taskspace->d2, taskspace->d3, taskspace->d4};
//	double delta[4] = {(double)(roll/100.00), (double)(x/100.00), (double)(y/100.00), (double)(z/100.00)};
	cartesianJog(qi_all, delta);
	taskspace->q1 = get_cartesian_q1();
	taskspace->q2 = get_cartesian_q2();
	taskspace->q3 = get_cartesian_q3();
	taskspace->q4 = get_cartesian_q4();

	double q[4] = {taskspace->q1, taskspace->q2, taskspace->q3, taskspace->q4};
	forwardKinematic(q);

	Stepper_SetTraget(1, to_degree(taskspace->q1));
	Stepper_SetTraget(2, to_degree(taskspace->q2));
	Stepper_SetTraget(3, abs(taskspace->q3));

	taskspace->q1 = to_radian((double)Stepper_targetPosition_real(1));
	taskspace->q2 = to_radian((double)Stepper_targetPosition_real(2));
	taskspace->q3 = (double)Stepper_targetPosition_real(3);
	taskspace->q4 = to_radian((double)0.0);
	double qq[4] = {taskspace->q1, taskspace->q2, taskspace->q3, taskspace->q4};
	forwardKinematic(qq);

}

void update_FK_real(){
	double qi_all[4] = {to_radian((double)Stepper_currentPosition_real(1)), to_radian((double)Stepper_currentPosition_real(2)), (double)Stepper_currentPosition_real(3), to_radian((double)0.0)};
	forwardKinematic(qi_all);
}

//void update_tarjectory(){
//
//}

void run_tarjectory(){
	taskspace_state * taskspace = &taskspaces[0];
//	taskspace->q1 = to_radian((double)Stepper_targetPosition_real(1));
//	taskspace->q2 = to_radian((double)Stepper_targetPosition_real(2));
//	taskspace->q3 = (double)Stepper_targetPosition_real(3);
//	taskspace->q4 = to_radian((double)0.0);
//	double q[4] = {taskspace->q1, taskspace->q2, taskspace->q3, taskspace->q4};
//	forwardKinematic(q);
	taskspace->t1 += 0.0001;
	double x_target = 200 + cos(0.2*taskspace->t1)*50;
	double y_target = 200 + cos(0.2*taskspace->t1)*50;
//	taskspace->t1 = x_target;
	taskspace->t2 = y_target;
//	double x_target = get_fk_X() - cos(2*0.0001)*10;
//	double y_target = get_fk_Y() - cos(2*0.0001)*10;
	double xyz[3] = {x_target, y_target, 0};

	IK(xyz, 0, 1);
	Stepper_SetTraget(1, to_degree(get_ik_q1()));
	Stepper_SetTraget(2, to_degree(get_ik_q2()));
}

