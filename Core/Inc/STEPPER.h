// File name: STEPPER.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS
// Modify from StepperHub: https://github.com/omuzychko/StepperHub

#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32h7xx_hal.h"

#define NUM_STEPPER 5 // amount motor to used
#define DEFAULT_MIN_SPEED 1
#define DEFAULT_MAX_SPEED 40000
#define ENCODER1_TO_ANGLE 36000.00/8071.00
#define ANGLE_TO_ENCODER1 8071.00/36000.00
#define ENCODER2_TO_ANGLE 36000.00/15369.00
#define ANGLE_TO_ENCODER2 15369.00/36000.00
#define ENCODER3_TO_SCALAR 10650.00/18530.00
#define SCALAR_TO_ENCODER3 18530.00/10650.00
//#define ANGLE_TO_ENCODER 36000/8192 //8192 from encoder mode PPR x 4, 36000 from 360.00 degrees * 100
//#define SCALAR_TO_ENCODER 100/8192 //100 measure mm
//#define ENCODER_TO_ANGLE 8192/36000 //8192 from encoder mode PPR x 4, 36000 from 360.00 degrees * 100
//#define ENCODER_TO_SCALAR 8192/100 //100 measure mm
#define OFFSET1 34070 // offset count encoder1
#define OFFSET2 22510 // offset count encoder2
#define OFFSET3 30000 // offset count encoder3
#define OFFSET 30000 // offset count encoder
//q1 encoder 8192 per 360 degree
//q2 encoder 15369 per 360 degree
//q3 encoder 18530 per 106.5 mm

//int AllHome_status = 0;

typedef enum {
    SS_UNDEFINED         = 0x00,
    SS_RUNNING_BACKWARD  = 0x01,
    SS_RUNNING_FORWARD   = 0x02,
    SS_STARTING          = 0x04,
    SS_BREAKING          = 0x10,
    SS_BREAKCORRECTION   = 0x20,
    SS_STOPPED           = 0x80
} stepper_status;

typedef enum {
    M_ANGLE         = 0x00,
    M_SCALAR	    = 0x01,
} stepper_mode;

typedef enum {
	SERR_OK                     = 0,
	SERR_NOMORESTATESAVAILABLE  = 1,
	SERR_MUSTBESTOPPED          = 2,
	SERR_STATENOTFOUND          = 3,
	SERR_LIMIT                  = 4
} stepper_error;

typedef struct{
	//number
	int number;

	//Timer Channel
	TIM_HandleTypeDef * STEP_TIMER;
	uint32_t STEP_CHANNEL;

	//GPIO pin
	GPIO_TypeDef * DIR_GPIO;
	volatile uint16_t DIR_PIN;

	//Prescaller
	volatile int32_t stepPrescaller;

	//Setup Speed
	volatile int32_t minSpeed;
	volatile int32_t maxSpeed;

	volatile float_t minPosition;
	volatile float_t maxPosition;

	//Setup acceleration
	volatile int32_t acceleration;

	//
	volatile int32_t currentSpeed;

	volatile int32_t currentPulse;

	volatile int32_t targetPosition;

	volatile int32_t currentPosition;

	volatile float_t targetPosition_real;

	volatile float_t currentPosition_real;

	volatile int32_t stepCtrlPrescaller;

	volatile int32_t stepCtrlPrescallerTicks;

	volatile int32_t breakInitiationSpeed;

	volatile stepper_status status;

	volatile stepper_mode modeStepper;

	volatile int8_t home_status;

	volatile int dir;


}stepper_state;

extern uint32_t STEP_TIMER_CLOCK;
extern uint32_t STEP_CONTROLLER_PERIOD_US;

stepper_error Stepper_Setup(int num, TIM_HandleTypeDef * stepTimer, uint32_t stepChannel, GPIO_TypeDef * dirGPIO, uint16_t dirPIN, stepper_mode mode);

void Stepper_SetStepTimer(stepper_state * stepper);

stepper_error Stepper_DefaultState(int num);

stepper_error Stepper_SetMinPosition(int num, float_t value);

stepper_error Stepper_SetMaxPosition(int num, float_t value);

stepper_error Stepper_SetMinSpeed(int num, uint16_t value);

stepper_error Stepper_SetMaxSpeed(int num, uint16_t value);

stepper_error Stepper_SetTraget(int num, float_t value);

stepper_error Stepper_SetSpeed(int num, int32_t value);

void Stepper_Direction(stepper_state * stepper);

void enable_Stepper_OE();

void disable_Stepper_OE();

void Stepper_runStep(int num);

void Stepper_StartStop(int num, uint8_t j);

stepper_status Stepper_status(int num);

void Stepper_updateHome(int num, int value);

void Stepper_SetHome(int num, int dir, int on);

int32_t Stepper_currentPosition(int num);

float_t Stepper_currentPosition_real(int num);

float_t Stepper_targetPosition_real(int num);

float_t encoder_to_joint(int num, int32_t value);

int32_t joint_to_encoder(int num, float_t value);

double to_radian(double value);

double to_degree(double value);

#endif /* __STEPPER_H */
