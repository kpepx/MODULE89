// File name: STEPPER.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS
// Modify from StepperHub: https://github.com/omuzychko/StepperHub

#include "STEPPER.h"
#include "math.h"
#include "QEI.h"
#include "PID.h"

static stepper_state steppers[NUM_STEPPER];

//Setup variable to use in stepper
stepper_error Stepper_Setup(int num, TIM_HandleTypeDef * stepTimer, uint32_t stepChannel, GPIO_TypeDef * dirGPIO, uint16_t dirPIN, stepper_mode mode){
	stepper_state * stepper = &steppers[num];
	stepper->number = num;
	stepper->STEP_TIMER = stepTimer;
	stepper->STEP_CHANNEL = stepChannel;
	stepper->DIR_GPIO = dirGPIO;
	stepper->DIR_PIN = dirPIN;
	stepper->modeStepper = mode;
	return SERR_OK;
}

//Update Frequency PWM and set duty 50%
void Stepper_SetStepTimer(stepper_state * stepper){
	if (stepper -> STEP_TIMER != NULL && stepper -> STEP_TIMER -> Instance != NULL){
		//    TIM_TypeDef * timer = stepper -> STEP_TIMER -> Instance;
		uint32_t prescaler = 0;
		uint32_t timerTicks = STEP_TIMER_CLOCK / stepper -> currentSpeed;

		if (timerTicks > 0xFFFF) {
			// calculate the minimum prescaler
			prescaler = timerTicks/0xFFFF;
			timerTicks /= (prescaler + 1);
		}
		stepper -> STEP_TIMER -> Instance -> PSC = prescaler;
		stepper -> STEP_TIMER -> Instance -> ARR = timerTicks;
		stepper -> STEP_TIMER -> Instance -> CCR1 = timerTicks/2;
	}
}

stepper_error Stepper_DefaultState(int num){
	stepper_state * stepper = &steppers[num];
	stepper -> status = SS_STOPPED;
	stepper -> minSpeed = DEFAULT_MIN_SPEED;
	stepper -> maxSpeed = DEFAULT_MAX_SPEED;
	stepper -> currentSpeed = stepper -> minSpeed;

	stepper -> targetPosition = OFFSET;
//	stepper -> currentPosition = 0;

	Stepper_SetStepTimer(stepper);
	HAL_TIM_PWM_Start(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
	enable_Stepper_OE();

	return SERR_OK;
}

stepper_error Stepper_SetMinPosition(int num, uint16_t value){
	stepper_state * stepper = &steppers[num];
	stepper->minPosition = value;
}

stepper_error Stepper_SetMaxPosition(int num, uint16_t value){
	stepper_state * stepper = &steppers[num];
	stepper->maxPosition = value;
}

stepper_error Stepper_SetMinSpeed(int num, uint16_t value){
	stepper_state * stepper = &steppers[num];
	stepper->minSpeed = value;
}

stepper_error Stepper_SetMaxSpeed(int num, uint16_t value){
	stepper_state * stepper = &steppers[num];
	stepper->maxSpeed = value;
}

stepper_error Stepper_SetTraget(int num, uint16_t value){
	stepper_state * stepper = &steppers[num];
	if(stepper->status != SS_STOPPED){
		if(value<stepper->minPosition){
			return SERR_LIMIT;
		}
		else if (value>stepper->maxPosition) {
			return SERR_LIMIT;
		}
		else {
			if(stepper->modeStepper == M_ANGLE){
				stepper->targetPosition = value*ANGLE_TO_ENCODER + OFFSET;
			}
			else {
				stepper->targetPosition = value*SCALAR_TO_ENCODER + OFFSET;
			}
			stepper->status = SS_STARTING;
		}
	}
	return SERR_OK;
}

stepper_error Stepper_SetSpeed(int num, int32_t value){
	stepper_state * stepper = &steppers[num];
	if(value<stepper->minSpeed){
		stepper->currentSpeed = stepper->minSpeed;
	}
	else if (value>stepper->maxSpeed) {
		stepper->currentSpeed = stepper->maxSpeed;
	}
	else{
		stepper->currentSpeed = value;
	}
}

void Stepper_Direction(stepper_state * stepper){
	int32_t input = calculator(stepper->number, Get_Value_Encoder(stepper->number), stepper->targetPosition);
	Stepper_SetSpeed(stepper->number, abs(input));
	if(input<0){
		stepper->status = SS_RUNNING_FORWARD;
		stepper->DIR_GPIO->BSRR = (uint32_t)stepper->DIR_PIN << (16U); //BSRR change pin to set/reset
	}
	else {
		stepper->status = SS_RUNNING_BACKWARD;
		stepper->DIR_GPIO->BSRR = stepper->DIR_PIN; //BSRR change pin to set/reset
	}
}

void enable_Stepper_OE(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
}

void disable_Stepper_OE(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
}

void Stepper_runStep(int num){
	stepper_state * stepper = &steppers[num];
	if(stepper->status != SS_STOPPED){
//		enable_Stepper_OE();
		HAL_TIM_PWM_Start(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
		if(stepper->home_status){
			Stepper_Direction(stepper);
			Stepper_SetStepTimer(stepper);
			//HAL_TIM_PWM_Start(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
		}
		else {

		}
	}
	else{
		HAL_TIM_PWM_Stop(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
//		disable_Stepper_OE();
	}
}

void Stepper_StartStop(int num, uint8_t j){
	stepper_state * stepper = &steppers[num];
	if(j == 1){
		stepper->status = SS_STARTING;
	}
	else {
		stepper->status = SS_STOPPED;
	}
}

void Stepper_updateHome(int num, int value){
	stepper_state * stepper = &steppers[num];
	if(stepper->home_status == 0){
		Set_Encoder_Zero(num);
		stepper->home_status = value;
		Stepper_DefaultState(num);
		stepper -> status = SS_STARTING;
	}
}

int8_t Stepper_Checkhome(int num){
	stepper_state * stepper = &steppers[num];
	return stepper->home_status;
}

void Stepper_SetHome(int num, int dir, int on){
	stepper_state * stepper = &steppers[num];
	if(on){
		stepper->home_status = 0;
		stepper->DIR_GPIO->BSRR = stepper->DIR_PIN; //BSRR change pin to set/reset
		stepper -> STEP_TIMER -> Instance -> PSC = 5;
		stepper -> STEP_TIMER -> Instance -> ARR = 25000;
		stepper -> STEP_TIMER -> Instance -> CCR1 = 25000/2;
	}
}
