// File name: STEPPER.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS
// Modify from StepperHub: https://github.com/omuzychko/StepperHub

#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32h7xx_hal.h"

#define NUM_STEPPER 5
#define DEFAULT_MIN_SPEED 1
#define DEFAULT_MAX_SPEED 400000
#define ANGLE_TO_ENCODER 8192/36000
#define SCALAR_TO_ENCODER 8192/100
#define OFFSET 15000

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
	uint16_t DIR_PIN;

	//Prescaller
	volatile int32_t stepPrescaller;

	//Setup Speed
	volatile int32_t minSpeed;
	volatile int32_t maxSpeed;

	volatile int32_t minPosition;
	volatile int32_t maxPosition;

	//Setup acceleration
	volatile int32_t acceleration;

	//
	volatile int32_t currentSpeed;

	volatile int32_t currentPulse;

	volatile int32_t targetPosition;

	volatile int32_t stepCtrlPrescaller;

	volatile int32_t stepCtrlPrescallerTicks;

	volatile int32_t breakInitiationSpeed;

	volatile int32_t currentPosition;

	volatile stepper_status status;

	volatile stepper_mode modeStepper;

	volatile int8_t home_status;


}stepper_state;

extern uint32_t STEP_TIMER_CLOCK;
extern uint32_t STEP_CONTROLLER_PERIOD_US;

stepper_error setupStepper(int num, TIM_HandleTypeDef * stepTimer, uint32_t stepChannel, GPIO_TypeDef * dirGPIO, uint16_t dirPIN, stepper_mode mode);

void Stepper_SetStepTimer(stepper_state * stepper);

stepper_error Stepper_DefaultState(int num);

stepper_error Stepper_SetMinPosition(int num, uint16_t value);

stepper_error Stepper_SetMaxPosition(int num, uint16_t value);

stepper_error Stepper_SetMinSpeed(int num, uint16_t value);

stepper_error Stepper_SetMaxSpeed(int num, uint16_t value);

stepper_error Stepper_SetTraget(int num, uint16_t value);

stepper_error Stepper_SetSpeed(int num, int32_t value);

void Stepper_Direction(stepper_state * stepper);

void enable_Stepper_OE();

void disable_Stepper_OE();

void Stepper_runStep(int num);

void Stepper_StartStop(int num, uint8_t j);

void Stepper_updateHome(int num, int value);

#endif /* __STEPPER_H */
