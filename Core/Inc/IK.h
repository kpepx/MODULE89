// File name: IK.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __IK_H
#define __IK_H

#include "stm32h7xx_hal.h"

#define NUM_IK 1

typedef struct{
	int number;
	volatile int32_t q1;
	volatile int32_t q2;
	volatile int32_t q3;
	volatile int32_t q4;


}ik_state;

void updateIK(int32_t x, int32_t y, int32_t z, int32_t r);


#endif /* __IK_H */
