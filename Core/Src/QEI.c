// File name: QEI.c
// Author: KRITTAPAK
// Project Name: Module8-9
// Group name: ISUS

#include "QEI.h"

//if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))
//	  	  {
//	  	     sprintf(MSG, "Encoder Switch Released, Encoder Ticks = %d\n\r", ((TIM2->CNT)>>2));
//	  	     HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
//	  	  }
//	  	 else
//	  	  {
//	  	     sprintf(MSG, "Encoder Switch Pressed,  Encoder Ticks = %d\n\r", ((TIM2->CNT)>>2));
//	  	     HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
//	  	  }
//	  	 HAL_Delay(100);
//
//	  	 //PWM Code 65535 pre loop
//	  	 TIM8->CCR1 = CH1_DC;
//	  	 CH1_DC += 1;
