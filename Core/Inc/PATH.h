// File name: PATH.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __PATH_H
#define __PATH_H

#include "stm32h7xx_hal.h"

#define NUM_PATH 2
#define PICK 1
#define PLACE 2

typedef struct{
	int check;
	int type;
	int row, column;
	double x, y;

}path_state;

void path(int32_t row, int32_t column,int32_t type);

double find_time(int32_t xi, int32_t yi, int32_t xf, int32_t yf);

void update_path();

#endif /* __PATH_H */
