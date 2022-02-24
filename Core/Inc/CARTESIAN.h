// File name: CARTESIAN.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __CARTESIAN_H
#define __CARTESIAN_H

#include "stm32h7xx_hal.h"

#define NUM_CARTESIAN 1

typedef struct{
	int number;
	volatile int32_t X;
	volatile int32_t Y;
	volatile int32_t Z;


}cartesian_state;

void updateXYZ(int32_t joint1, int32_t joint2, int32_t joint3, int32_t joint4);

void updateJoint(int32_t x, int32_t y, int32_t z);


#endif /* __CARTESIAN_H */
