#ifndef __TRAJECTORY_CIRCLE_H
#define __TRAJECTORY_CIRCLE_H

#include <stddef.h>
#include <stdlib.h>

#include "stm32h7xx_hal.h"

#define NUM_TRAJECTORY_CIRCLE 4

typedef struct{
	volatile double x_circle;
	volatile double y_circle;
	volatile double x_circle_target;
	volatile double y_circle_target;
	volatile double r_circle;
	volatile double d_circle; //ref horizontal
	volatile double w_circle;
	volatile double t_circle;
	volatile double t_sum_circle;
	volatile double d_robot;
	volatile double z_robot;

}trajectory_circle_state;


void update_circle(int32_t row, int32_t column, int32_t w, int32_t t);


#endif /* __TRAJECTORY_CIRCLE_H */
