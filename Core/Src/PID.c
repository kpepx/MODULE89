// File name: PID.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "PID.h"

static pid_state pids[NUM_PID];

void setupPID(int num, float dt, float vmin, float vmax, float kp, float ki, float kd){
	pid_state * pid = &pids[num];
	pid->number = num;
	pid->Dt = dt;
	pid->Vmax = vmax;
	pid->Vmin = vmin;
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}


float calculator(int num, int input, float setpoint){
	pid_state * pid = &pids[num];
	// Calculate error
	pid->error = setpoint - input;

	// Proportional term
	float Pout = pid->Kp * pid->error;

	// Integral term
	pid->integral += pid->error * pid->Dt;
	float Iout = pid->Ki * pid->integral;
	if (pid->integral > 255) {
		pid->integral = 255;
	}
	if (pid->integral < -255) {
		pid->integral = -255;
	}
	// Derivative term
	float derivative = (pid->error - pid->error_pre) / pid->Dt;
	float Dout = pid->Kd * derivative;

	// Calculate total output
	float output = Pout + Iout + Dout;

	// Restrict to max/min
	if( output > pid->Vmax){
	     output = pid->Vmax;}
	else if( output < pid->Vmin){
	     output = pid->Vmin;}

	// Save error to previous error
	pid->error_pre = pid->error;

	return output;
}
