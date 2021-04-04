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

//void updateTimerStepper(int num, int period, int pulse){
//	stepper_state * stepper = &steppers[num];
//	__HAL_TIM_SET_COMPARE(stepper->STEP_TIMER, stepper->STEP_CHANNEL, period);
//	__HAL_TIM_SET_COUNTER(stepper->STEP_TIMER, pulse);
//}
//
//void setStepTimer(stepper_state * stepper){
//	TIM_TypeDef * timer = stepper->STEP_TIMER->Instance;
//	uint32_t prescale = 0;
//	uint32_t timerTicks = STEP
//}

void SetStepTimer(stepper_state * stepper){
  if (stepper -> STEP_TIMER != NULL && stepper -> STEP_TIMER -> Instance != NULL){
    TIM_TypeDef * timer = stepper -> STEP_TIMER -> Instance;
    uint32_t prescaler = 0;
    uint32_t timerTicks = STEP_TIMER_CLOCK / stepper -> currentTimer;

    if (timerTicks > 0xFFFF) {
        // calculate the minimum prescaler
        prescaler = timerTicks/0xFFFF;
        timerTicks /= (prescaler + 1);
    }

    timer -> PSC = prescaler;
    timer -> ARR = timerTicks;
  }
}

void DecrementStepper(stepper_state * stepper){
    if (stepper -> currentTimer > stepper -> minSpeed){
        stepper -> currentTimer -=  stepper -> acceleration;
        SetStepTimer(stepper);
    }
}

void IncrementStepper(stepper_state * stepper){
    if (stepper -> currentTimer < stepper -> maxSpeed) {
        stepper -> currentTimer +=  stepper -> acceleration;
        SetStepTimer(stepper);
    }
}


//__HAL_TIM_SET_COUNTER(stepper_sta);
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


//Calculate Frequency
//Fpwm = Fclk/((ARR+1)*(PSC+1))
//ARR

//Calculate Duty Cycle
//Duty = CCRx/ARRx
//CCR

//Change Pulse PWM width
//Change duty Cycle

//TIM2->ARR = <value>; // Period
//TIM2->CCR1 = <othervalue>; // guess CCR1 means Channel 1, Duty Cycle

//void user_pwm_setvalue(uint16_t value)
//{
//    TIM_OC_InitTypeDef sConfigOC;
//
//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = value;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//}
//
//void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
//{
// HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
// TIM_OC_InitTypeDef sConfigOC;
// timer.Init.Period = period; // set the period duration
// HAL_TIM_PWM_Init(&timer); // reinititialise with new period value
// sConfigOC.OCMode = TIM_OCMODE_PWM1;
// sConfigOC.Pulse = pulse; // set the pulse duration
// sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
// sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
// HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
// HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
//}
