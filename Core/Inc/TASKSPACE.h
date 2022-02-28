// File name: TASKSPACE.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __TASKSPACE_H
#define __TASKSPACE_H

#include "stm32h7xx_hal.h"

#define NUM_TASKSPACE 1

typedef struct{
	int number;
	volatile int32_t X;
	volatile int32_t Y;
	volatile int32_t Z;


}taskspace_state;

void updateXYZ(int32_t joint1, int32_t joint2, int32_t joint3, int32_t joint4);

void updateJoint(int32_t x, int32_t y, int32_t z);


#endif /* __TASKSPACE_H */
