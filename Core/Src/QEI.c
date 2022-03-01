// File name: QEI.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "QEI.h"

//Encoder TIM1 use for Joint 1
//Encoder TIM3 use for Joint 2
//Encoder TIM4 use for Joint 3
//TIM2 is Spare

static encoder_state encoders[NUM_ENCODER];

void enable_Encoder_OE(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
}

void disable_Encoder_OE(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
}

void Encoder_Start(int num, TIM_HandleTypeDef * qeiTimer, uint32_t qeiChannel){ //Start All Encoder
	encoder_state * encoder = &encoders[num];
	HAL_TIM_Encoder_Start(qeiTimer, qeiChannel);
	encoder->number = num;
	encoder->QEI_TIMER = qeiTimer;
	encoder->QEI_CHANNEL = qeiChannel;
	enable_Encoder_OE();
	Set_Encoder_Zero(num);
}

uint32_t Get_Value_Encoder(int num){ //Read Encoder Select by input num
	encoder_state * encoder = &encoders[num];
	if(__HAL_TIM_GET_COUNTER(encoder->QEI_TIMER) >= 0){
		return __HAL_TIM_GET_COUNTER(encoder->QEI_TIMER); //return value to use
	}
}

void Set_Encoder_Zero(int num){
	encoder_state * encoder = &encoders[num];
	__HAL_TIM_SET_COUNTER(encoder->QEI_TIMER, OFFSET);
}
