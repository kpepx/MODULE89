// File name: STEPPER.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS
// Modify from StepperHub: https://github.com/omuzychko/StepperHub

#include "STEPPER.h"
#include "math.h"
#include "QEI.h"
#include "PID.h"
#include "TASKSPACE.h"

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

stepper_error Stepper_SetMinPosition(int num, float_t value){
	stepper_state * stepper = &steppers[num];
	stepper->minPosition = value;
	return SERR_OK;
}

stepper_error Stepper_SetMaxPosition(int num, float_t value){
	stepper_state * stepper = &steppers[num];
	stepper->maxPosition = value;
	return SERR_OK;
}

stepper_error Stepper_SetMinSpeed(int num, uint16_t value){
	stepper_state * stepper = &steppers[num];
	stepper->minSpeed = value;
	return SERR_OK;
}

stepper_error Stepper_SetMaxSpeed(int num, uint16_t value){
	stepper_state * stepper = &steppers[num];
	stepper->maxSpeed = value;
	return SERR_OK;
}

stepper_error Stepper_SetTraget(int num, float_t value){
	//value is mm or degree unit
	stepper_state * stepper = &steppers[num];
	if(stepper->status != SS_STOPPED){
		if(value<stepper->minPosition){
			stepper->targetPosition_real = stepper->minPosition;
			stepper->targetPosition = joint_to_encoder(num, (stepper->minPosition)*100.00) + OFFSET;
			return SERR_LIMIT;
		}
		else if (value>stepper->maxPosition) {
			stepper->targetPosition_real = stepper->maxPosition;
			stepper->targetPosition = joint_to_encoder(num, (stepper->maxPosition)*100.00) + OFFSET;
			return SERR_LIMIT;
		}
		else {
			stepper->targetPosition_real = value; //input mm or degree unit
			stepper->targetPosition = joint_to_encoder(num, value*100.00) + OFFSET; //convert to encoder count
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
	if(input>0){
		if(stepper->number == 1){
			stepper->status = SS_RUNNING_BACKWARD;
			stepper->DIR_GPIO->BSRR = stepper->DIR_PIN; //BSRR change pin to set/reset
		}
		if(stepper->number == 2){
			stepper->status = SS_RUNNING_FORWARD;
			stepper->DIR_GPIO->BSRR = (uint32_t)stepper->DIR_PIN << (16U); //BSRR change pin to set/reset
		}
		if(stepper->number == 3){
			stepper->status = SS_RUNNING_FORWARD;
			stepper->DIR_GPIO->BSRR = stepper->DIR_PIN; //BSRR change pin to set/reset
		}

	}
	else {
		if(stepper->number == 1){
			stepper->status = SS_RUNNING_FORWARD;
			stepper->DIR_GPIO->BSRR = (uint32_t)stepper->DIR_PIN << (16U); //BSRR change pin to set/reset
		}
		if(stepper->number == 2){
			stepper->status = SS_RUNNING_BACKWARD;
			stepper->DIR_GPIO->BSRR = stepper->DIR_PIN; //BSRR change pin to set/reset
		}
		if(stepper->number == 3){
			stepper->status = SS_RUNNING_BACKWARD;
			stepper->DIR_GPIO->BSRR = (uint32_t)stepper->DIR_PIN << (16U); //BSRR change pin to set/reset
		}
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
	Stepper_currentPosition(num);
	Stepper_currentPosition_real(num);
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
		Set_Encoder_Zero(num, OFFSET);
		stepper->home_status = value;
		Stepper_DefaultState(num);
		stepper -> status = SS_STARTING;
	}
}

int8_t Stepper_Checkhome(int num){
	stepper_state * stepper = &steppers[num];
	return stepper-> home_status;
}

void Stepper_SetHome(int num, int dir, int on){
	stepper_state * stepper = &steppers[num];
	if(on){
		if(num == 3){
			stepper-> home_status = 0;
			stepper->DIR_GPIO->BSRR = (uint32_t)stepper->DIR_PIN << (16U); //BSRR change pin to set/reset
			stepper -> STEP_TIMER -> Instance -> PSC = 25;
			stepper -> STEP_TIMER -> Instance -> ARR = 64000;
			stepper -> STEP_TIMER -> Instance -> CCR1 = 64000/2;
		}
		else{
			stepper-> home_status = 0;
			stepper-> DIR_GPIO->BSRR = stepper->DIR_PIN; //BSRR change pin to set/reset
			stepper -> STEP_TIMER -> Instance -> PSC = 25;
			stepper -> STEP_TIMER -> Instance -> ARR = 64000;
			stepper -> STEP_TIMER -> Instance -> CCR1 = 64000/2;
		}
	}
}

int32_t Stepper_currentPosition(int num){
	//update current real encoder of robot
	stepper_state * stepper = &steppers[num];
	stepper->currentPosition = Get_Value_Encoder(num);
	return stepper->currentPosition;
}

float_t Stepper_currentPosition_real(int num){
	//update current real position of robot
	stepper_state * stepper = &steppers[num];
//	stepper->currentPosition_real = encoder_to_joint(num, abs(Stepper_currentPosition(num)-OFFSET))/100.00;
	stepper->currentPosition_real = encoder_to_joint(num, Stepper_currentPosition(num)-OFFSET)/100.00;
	return stepper->currentPosition_real;
}

float_t Stepper_targetPosition_real(int num){
	//send target real position of robot
	stepper_state * stepper = &steppers[num];
	return stepper->targetPosition_real;
}

// convert encoder to degree and scalar
float_t encoder_to_joint(int num, int32_t value){
	//convert to mm or degree
	float_t ans;
	if(num == 1){
		ans = value*ENCODER1_TO_ANGLE;
	}
	else if(num == 2){
		ans = value*ENCODER2_TO_ANGLE;
	}
	else if(num == 3){
		ans = value*ENCODER3_TO_SCALAR;
	}
	return ans;
}

// convert degree and scalar to encoder
int32_t joint_to_encoder(int num, float_t value){
	//convert to encoder count
	int32_t ans;
	if(num == 1){
		ans = value*ANGLE_TO_ENCODER1;
	}
	else if(num == 2){
		ans = value*ANGLE_TO_ENCODER2;
	}
	else if(num == 3){
		ans = value*SCALAR_TO_ENCODER3;
	}
	return ans;
}
