// File name: STEPPER.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS
// Modify from StepperHub: https://github.com/omuzychko/StepperHub

#include "STEPPER.h"
#include "math.h"

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

//Set limit max/min Position
stepper_error Stepper_SetMaxMinPosition(int num, int32_t minPos, int32_t maxPos){
	stepper_state * stepper = &steppers[num];
	stepper->minPosition = minPos;
	stepper->maxPosition = maxPos;
	return SERR_OK;
}

//Update Frequency PWM and set duty 50%
void Stepper_SetStepTimer(stepper_state * stepper){
  if (stepper -> STEP_TIMER != NULL && stepper -> STEP_TIMER -> Instance != NULL){
    TIM_TypeDef * timer = stepper -> STEP_TIMER -> Instance;
    uint32_t prescaler = 0;
    uint32_t timerTicks = STEP_TIMER_CLOCK / stepper -> currentSpeed;

    if (timerTicks > 0xFFFF) {
        // calculate the minimum prescaler
        prescaler = timerTicks/0xFFFF;
        timerTicks /= (prescaler + 1);
    }

    timer -> PSC = prescaler;
    timer -> ARR = timerTicks;
    __HAL_TIM_SET_COMPARE(stepper->STEP_TIMER, stepper->STEP_CHANNEL, timerTicks/2);
  }
}

stepper_error Stepper_DefaultState(int num){
	stepper_state * stepper = &steppers[num];
//	if(stepper == NULL){
//		stepper -> number = num;
//		stepper -> status = SS_STOPPED;
//	}
//	else if (!(stepper -> status & SS_STOPPED)) {
//		return SERR_MUSTBESTOPPED;
//	}
	stepper -> status = SS_STOPPED;
	stepper -> minSpeed = DEFAULT_MIN_SPEED;
	stepper -> maxSpeed = DEFAULT_MAX_SPEED;
	stepper -> currentSpeed = stepper -> minSpeed;

	stepper -> targetPosition = 0;
	stepper -> currentPosition = 0;

	Stepper_SetStepTimer(stepper);

	return SERR_OK;
}

//void Stepper_updateAcceleration(stepper_state * stepper){
//	float fAccSPS = 0.8f * STEP_CONTROLLER_PERIOD_US * stepper->minSpeed / 1000000.0f;
//	if (fAccSPS > 10.0f) {
////		stepper->stepCtrlPrescallerTicks =
//	    stepper->stepCtrlPrescaller = 1;
//	    stepper->acceleration = fAccSPS; // In worst case scenario, like 10.99 we will get 10% less (0.99 out of almost 11.00) acceleration
//	} else {
//	    // Here it is better to use prescaller
//	    uint32_t prescalerValue = 1;
//	    float prescaledAcc = fAccSPS;
//	    float remainder = prescaledAcc - (uint32_t)prescaledAcc;
//
//	    while (prescaledAcc < 0.9f || (0.1f < remainder & remainder < 0.9f)) {
//	        prescalerValue++;
//	        prescaledAcc += fAccSPS;
//	        remainder = prescaledAcc - (uint32_t)prescaledAcc;
//	    }
//
////	    stepper->stepCtrlPrescallerTicks =
//	    stepper->stepCtrlPrescaller = prescalerValue;
//	    stepper -> acceleration = prescaledAcc;
//
//	    // Round up if at upper remainder
//	    if (remainder > 0.9f)
//	        stepper -> acceleration += 1;
//	}
//}
//
////Update Frequency PWM and set duty 50%
//void Stepper_SetStepTimer(stepper_state * stepper){
//  if (stepper -> STEP_TIMER != NULL && stepper -> STEP_TIMER -> Instance != NULL){
//    TIM_TypeDef * timer = stepper -> STEP_TIMER -> Instance;
//    uint32_t prescaler = 0;
//    uint32_t timerTicks = STEP_TIMER_CLOCK / stepper -> currentSpeed;
//
//    if (timerTicks > 0xFFFF) {
//        // calculate the minimum prescaler
//        prescaler = timerTicks/0xFFFF;
//        timerTicks /= (prescaler + 1);
//    }
//
//    timer -> PSC = prescaler;
//    timer -> ARR = timerTicks;
//    timer -> CCR1 = timerTicks/2; //__HAL_TIM_SET_COMPARE(stepper->STEP_TIMER, stepper->STEP_CHANNEL, timerTicks/2);
//  }
//}
//
//void DecrementSpeed(stepper_state * stepper){
//    if (stepper -> currentSpeed > stepper -> minSpeed){
//        stepper -> currentSpeed -=  stepper -> acceleration;
//        Stepper_SetStepTimer(stepper);
//    }
//}
//
//void IncrementSpeed(stepper_state * stepper){
//    if (stepper -> currentSpeed < stepper -> maxSpeed) {
//        stepper -> currentSpeed +=  stepper -> acceleration;
//        Stepper_SetStepTimer(stepper);
//    }
//}
//
//
////Set to default
//stepper_error Stepper_DefaultState(int num){
//	stepper_state * stepper = &steppers[num];
//	if(stepper == NULL){
//		stepper -> number = num;
//		stepper -> status = SS_STOPPED;
//	}
//	else if (!(stepper -> status & SS_STOPPED)) {
//		return SERR_MUSTBESTOPPED;
//	}
//	stepper -> minSpeed = DEFAULT_MIN_SPEED;
//	stepper -> maxSpeed = DEFAULT_MAX_SPEED;
//	stepper -> currentSpeed = stepper -> minSpeed;
//
//	stepper -> targetPosition = 0;
//	stepper -> currentPosition = 0;
//
//	__HAL_TIM_SET_COMPARE(stepper->STEP_TIMER, stepper->STEP_CHANNEL, 0);
//	Stepper_updateAcceleration(stepper);
//	Stepper_SetStepTimer(stepper);
//
//	return SERR_OK;
//}
//
//int32_t GetStepDirectionUnit(stepper_state * stepper){
//    return (stepper->status & SS_RUNNING_BACKWARD) ? -1 : 1;
//}
//
//int64_t GetStepsToTarget(stepper_state * stepper) {
//    // returns absolute value of steps left to target
//    return ((int64_t)stepper->targetPosition - (int64_t)stepper->currentPosition) * GetStepDirectionUnit(stepper);
//}
//
//void Stepper_updateDirection(stepper_state * stepper){
////	stepper_state * stepper = &steppers[num];
//	if(stepper->currentPosition < stepper->targetPosition){
//		stepper->status = SS_RUNNING_FORWARD;
//		stepper->DIR_GPIO->BSRR = (uint32_t)stepper->DIR_PIN << 16u; //BSRR change pin to set/reset
//		stepper->currentPosition += GetStepDirectionUnit(stepper);
//	}
//	else if (stepper->currentPosition > stepper->targetPosition) {
//		stepper->status = SS_RUNNING_BACKWARD;
//		stepper->DIR_GPIO->BSRR = stepper->DIR_PIN;
//		stepper->currentPosition += GetStepDirectionUnit(stepper);
//	}
//	else if (stepper->currentPosition == stepper->targetPosition) {
//		stepper->status = SS_STOPPED;
//		HAL_TIM_PWM_Stop(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
//	}
//	if (GetStepsToTarget(stepper) <= 0 && stepper -> currentSpeed == stepper -> minSpeed) {
//		// We reached or passed through our target position at the stopping speed
//		stepper->status = SS_STOPPED;
//		HAL_TIM_PWM_Stop(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
//		//	printf("%c.stop:%d\r\n", stepper->number, stepper->currentPosition);
//	}
//}
//
//void Stepper_AllRunStepper(void){
//	Stepper_runStepper(1);
//	Stepper_runStepper(2);
//	Stepper_runStepper(3);
//}
//
//void Stepper_runStepper(stepper_state * stepper){
//	stepper_status status = stepper -> status;
//	if(status & SS_STOPPED){
//		if(stepper->targetPosition != stepper->currentPosition){
//			stepper->status = SS_STARTING;
//			HAL_TIM_PWM_Start(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
//		}
//	}
//	if(status & SS_STARTING){
//		Stepper_updateDirection(stepper);
//	}
//}
//
//void Stepper_updateAngle(stepper_state * stepper){
//
//}
//
//void Stepper_updatePulse(int num){
//	stepper_state * stepper = &steppers[num];
//	Stepper_runStepper(stepper);
////	if(stepper->modeStepper == M_ANGLE){
////		Stepper_updateAngle(stepper);
////	}
////	else if (stepper->modeStepper == M_SCALAR) {
//////		Stepper_updateScalar(stepper);
////	}
//
//}
//
//stepper_error Stepper_SetTargetPosition(int num, int32_t value){
//	stepper_state * stepper = &steppers[num];
//	if (stepper == NULL)
//		return SERR_STATENOTFOUND;
//	if(stepper->minPosition <= value && value <= stepper->maxPosition){
//		stepper->targetPosition = value;
//		stepper->status = SS_STARTING;
//		Stepper_updatePulse(1);
//	}
//	else{
//		stepper->status = SS_STOPPED;
//		HAL_TIM_PWM_Stop(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
//		return SERR_LIMIT;
//	}
//	return SERR_OK;
//
//}

	  //Ft = Fc/(Prescale+1)
	  //T = (1/Ft)(Cp+1) Cp preiod


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

