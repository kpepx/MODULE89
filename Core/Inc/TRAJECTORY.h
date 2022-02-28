// File name: TRAJECTORY.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include "stm32h7xx_hal.h"

#define NUM_TRAJECTORY 4

typedef struct{
	//number
	int number;
	volatile float32_t c0X;
	volatile float32_t c0Y;
	volatile float32_t c0Z;
	volatile float32_t c1X;
	volatile float32_t c1Y;
	volatile float32_t c1Z;
	volatile float32_t c2X;
	volatile float32_t c2Y;
	volatile float32_t c2Z;
	volatile float32_t c3X;
	volatile float32_t c3Y;
	volatile float32_t c3Z;


}trajectory_state;

void inputTarjectoryXYZ(float32_t pfX, float32_t vfX, float32_t pfY, float32_t vfY, float32_t pfZ, float32_t vfZ, float32_t Ts, float32_t sampling);
void updateTarjectoryXYZ();

#endif /* __TRAJECTORY_H */
