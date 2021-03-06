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
	if (pid->integral > 4095) {
		pid->integral = 4095;
	}
	if (pid->integral < -4095) {
		pid->integral = -4095;
	}
	// Derivative term
	float derivative = (pid->error - pid->error_pre) / pid->Dt;
	float Dout = pid->Kd * derivative;

	// Calculate total output
	pid->output = Pout + Iout + Dout;

	// Restrict to max/min
	if( pid->output > pid->Vmax){
		pid->output = pid->Vmax;}
	else if( pid->output < pid->Vmin){
		pid->output = pid->Vmin;}

	// Save error to previous error
	pid->error_pre = pid->error;

	return pid->output;
}
