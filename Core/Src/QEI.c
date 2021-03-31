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

void Encoder_Start(int num, TIM_HandleTypeDef * qeiTimer, uint32_t qeiChannel){ //Start All Encoder
	encoder_state * encoder = &encoders[num];
	HAL_TIM_Encoder_Start(qeiTimer, qeiChannel);
	encoder->number = num;
	encoder->QEI_TIMER = qeiTimer;
	encoder->QEI_CHANNEL = qeiChannel;
}

int Get_Value_Encoder(int num){ //Read Encoder Select by input num
	encoder_state * encoder = &encoders[num];
	return __HAL_TIM_GET_COUNTER(encoder->QEI_TIMER); //return value to use
}
