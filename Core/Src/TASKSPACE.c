// File name: TASKSPACE.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "TASKSPACE.h"
#include "STEPPER.h"

static taskspace_state taskspaces[NUM_CARTESIAN];

void updateXYZ(int32_t x, int32_t y, int32_t z){
	taskspace_state * taskspace = &taskspaces[0];
	taskspace->X = 0;
	taskspace->Y = 0;
	taskspace->Z = 0;
}

void updateJoint(int32_t joint1, int32_t joint2, int32_t joint3, int32_t joint4){
	taskspace_state * taskspace = &taskspaces[0];
	taskspace->X = 0;
	taskspace->Y = 0;
	taskspace->Z = 0;
//	Stepper_SetTraget(1, (serial->rPacket[6]<<8) + serial->rPacket[5]);
//	Stepper_SetTraget(2, (serial->rPacket[8]<<8) + serial->rPacket[7]);
//	Stepper_SetTraget(3, (serial->rPacket[10]<<8) + serial->rPacket[9]);
//	Servo_tragetPos(2, (serial->rPacket[12]<<8) + serial->rPacket[11]);
}
