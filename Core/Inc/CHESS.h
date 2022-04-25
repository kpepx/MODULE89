// File name: CHESS.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __CHESS_H
#define __CHESS_H

#include "stm32h7xx_hal.h"

#define NUM_CHESS 2
#define L 400.00
#define s 50.00
#define ENCODER_TO_ANGLE 36000.00/2000.00

typedef struct{
	volatile double degree_chess;
	volatile double x_chess, y_chess;
	volatile double degree_field;
	volatile double x_chess_real, y_chess_real;
	volatile double r;

}chess_state;

void updateChess(int32_t row, int32_t column);

double find_x(int32_t row);

double find_y(int32_t column);

double find_radius(double x, double y);

double find_degree(double x, double y);

double get_radius_circle();

double get_degree_chess();

double get_real_degree_chess();

double get_degree_field();

void set_field_zero();

double get_x_chess();

double get_y_chess();

#endif /* __CHESS_H */
