// File name: QEI.h
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#ifndef __QEI_H
#define __QEI_H

#include "stm32h7xx_hal.h"

#define NUM_ENCODER 5
#define OFFSET 15000

typedef struct{
	int number;

	TIM_HandleTypeDef * QEI_TIMER;
	uint32_t QEI_CHANNEL;

}encoder_state;

//Start Encoder
void Encoder_Start(int num, TIM_HandleTypeDef * qeiTimer, uint32_t qeiChannel);

//Get Value from Encoder
uint16_t Get_Value_Encoder(int num);

#endif /* __QEI_H */
