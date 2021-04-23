// File name: PID.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __PID_H
#define __PID_H

#define NUM_PID 5

typedef struct{
	//number
	int number;

	//time control loop
	volatile float Dt;

	//setup value ki kp kd
	volatile float Kp, Ki, Kd;

	//Set min-max speed
	volatile float Vmax, Vmin;

	volatile float output;

	//Variable use in calculator function
	volatile float error_pre, error, integral;
}pid_state;

//Setup PID
void setupPID(int num, float dt, float vmin, float vmax, float kp, float ki, float kd);

//Calculator PID
float calculator(int num, int input, float setpoint);


#endif /* __PID_H */
