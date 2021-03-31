// File name: STEPPER.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32h7xx_hal.h"

#define NUM_STEPPER 4

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

	//Setup acceleration
	volatile int32_t acceleration;


}stepper_state;

void setupStepper(int num, TIM_HandleTypeDef * stepTimer, uint32_t stepChannel, GPIO_TypeDef * dirGPIO, uint16_t dirPIN);

#endif /* __STEPPER_H */
