// File name: CHESS.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "CHESS.h"
#include "QEI.h"
#include <math.h>

static chess_state chesss[NUM_CHESS];

void updateChess(int32_t row, int32_t column){
	chess_state * chess = &chesss[0];
	chess->x_chess = find_x(row);
	chess->y_chess = find_y(column);
	chess->r = find_radius(chess->x_chess, chess->y_chess);
	chess->degree_chess = find_degree(chess->x_chess, chess->y_chess);
}

double find_x(int32_t row){
	if(row <= 4){
		return (-((5 - row)*L)/8.00) + (s/2.00);
	}
	else if(row >= 5){
		return (((row - 4)*L)/8.00) - (s/2.00);
	}
}

double find_y(int32_t column){
	if(column <= 4){
		return (-((5 - column)*L)/8.00) + (s/2.00);
	}
	else if(column >= 5){
		return (((column - 4)*L)/8.00) - (s/2.00);
	}
}

double find_radius(double x, double y){
	return sqrt(pow(x, 2) + pow(y, 2));
}

//ref 0 degree
double find_degree(double x, double y){
	double d = atan2(y, x) * (180.00/M_PI);
	if(x < 0 && y < 0){return 360.00 + d;}
	else if(x >= 0 && y < 0){return 360.00 + d;}
	else{return d;}

//    double d = atan(y/x)*(180.00/M_PI);
//    if(x >= 0 && y >= 0){return d;}
//    else if(x < 0 && y >= 0){return 180.00 + d;}
//    else if(x < 0 && y < 0){return 180.00 + d;}
//    else if(x >= 0 && y < 0){return 360.00 + d;}
}

double get_degree_chess(){ //ref
	chess_state * chess = &chesss[0];
	return chess->degree_chess;
}

double get_radius_circle(){
	chess_state * chess = &chesss[0];
	return chess->r;
}

double get_degree_field(){
	chess_state * chess = &chesss[0];
	chess->degree_field = (Get_Value_Encoder(4) * ENCODER_TO_ANGLE)/100.00;
	return chess->degree_field;
}

double get_real_degree_chess(){
	return ((int)((get_degree_field() + get_degree_chess())*100.00) % 36000)/100.00;
}

void set_field_zero(){
	Set_Encoder_Zero(4, 0);
}

double get_x_chess(){
	chess_state * chess = &chesss[0];
	return chess->r * cos(get_real_degree_chess());
}

double get_y_chess(){
	chess_state * chess = &chesss[0];
	return chess->r * sin(get_real_degree_chess());
}

