// File name: TASKSPACE.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __TASKSPACE_H
#define __TASKSPACE_H

#include "stm32h7xx_hal.h"

#define NUM_TASKSPACE 1

typedef struct{
	volatile double q1;
	volatile double q2;
	volatile double q3;
	volatile double q4;
	volatile double qi1;
	volatile double qi2;
	volatile double qi3;
	volatile double qi4;

	volatile double d1;
	volatile double d2;
	volatile double d3;
	volatile double d4;

	volatile double t1;
	volatile double t2;


}taskspace_state;

void updateXYZ(int32_t x, int32_t y, int32_t z);

void updateJoint(int32_t roll, int32_t x, int32_t y, int32_t z);

void run_tarjectory();

#endif /* __TASKSPACE_H */
