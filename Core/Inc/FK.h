// File name: FK.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __FK_H
#define __FK_H

#include "stm32h7xx_hal.h"

#define NUM_FK 1

typedef struct{
	int number;
	volatile int32_t X;
	volatile int32_t Y;
	volatile int32_t Z;
	volatile int32_t R;


}fk_state;

void updateFK(int32_t joint1, int32_t joint2, int32_t joint3, int32_t joint4);


#endif /* __FK_H */
