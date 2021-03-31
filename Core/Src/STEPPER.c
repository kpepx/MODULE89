// File name: STEPPER.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "STEPPER.h"

static stepper_state steppers[NUM_STEPPER];

void setupStepper(int num, TIM_HandleTypeDef * stepTimer, uint32_t stepChannel, GPIO_TypeDef * dirGPIO, uint16_t dirPIN){
	stepper_state * stepper = &steppers[num];
	HAL_TIM_PWM_Start(stepTimer, stepChannel);
	__HAL_TIM_SET_COMPARE(stepTimer, stepChannel, 0);
	stepper->number = num;
	stepper->STEP_TIMER = stepTimer;
	stepper->STEP_CHANNEL = stepChannel;
	stepper->DIR_GPIO = dirGPIO;
	stepper->DIR_PIN = dirPIN;
}

//  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
//  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);

	  //Ft = Fc/(Prescale+1)
	  //T = (1/Ft)(Cp+1) Cp preiod
//	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);//Clock wise rotation
//	  for(uint8_t duty=0; duty<99; duty++){
//		  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, duty);
//		  HAL_Delay(20);
//	  }
//	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);//Clock wise rotation
//	  	  for(uint8_t duty=0; duty<99; duty++){
//	  		  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, duty);
//	  		  HAL_Delay(20);
//	  	  }
