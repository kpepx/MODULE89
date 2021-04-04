/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_4
#define DIR2_GPIO_Port GPIOF
#define DIR1_Pin GPIO_PIN_5
#define DIR1_GPIO_Port GPIOF
#define Gripper_RX_Pin GPIO_PIN_6
#define Gripper_RX_GPIO_Port GPIOF
#define Gripper_TX_Pin GPIO_PIN_7
#define Gripper_TX_GPIO_Port GPIOF
#define Current_3_Pin GPIO_PIN_8
#define Current_3_GPIO_Port GPIOF
#define DIR4_Pin GPIO_PIN_10
#define DIR4_GPIO_Port GPIOF
#define Current_1_Pin GPIO_PIN_2
#define Current_1_GPIO_Port GPIOC
#define Current_2_Pin GPIO_PIN_3
#define Current_2_GPIO_Port GPIOC
#define QEI4A_Pin GPIO_PIN_5
#define QEI4A_GPIO_Port GPIOA
#define PLUSE3_Pin GPIO_PIN_6
#define PLUSE3_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define DIR3_Pin GPIO_PIN_8
#define DIR3_GPIO_Port GPIOE
#define QEI1A_Pin GPIO_PIN_9
#define QEI1A_GPIO_Port GPIOE
#define QEI1B_Pin GPIO_PIN_11
#define QEI1B_GPIO_Port GPIOE
#define PC_RX_Pin GPIO_PIN_12
#define PC_RX_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define PLUSE4_Pin GPIO_PIN_15
#define PLUSE4_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define QEI3A_Pin GPIO_PIN_12
#define QEI3A_GPIO_Port GPIOD
#define QEI3B_Pin GPIO_PIN_13
#define QEI3B_GPIO_Port GPIOD
#define PROXIMITY1_Pin GPIO_PIN_4
#define PROXIMITY1_GPIO_Port GPIOD
#define PROXIMITY1_EXTI_IRQn EXTI4_IRQn
#define PROXIMITY2_Pin GPIO_PIN_5
#define PROXIMITY2_GPIO_Port GPIOD
#define PROXIMITY2_EXTI_IRQn EXTI9_5_IRQn
#define PROXIMITY3_Pin GPIO_PIN_6
#define PROXIMITY3_GPIO_Port GPIOD
#define PROXIMITY3_EXTI_IRQn EXTI9_5_IRQn
#define PROXIMITY4_Pin GPIO_PIN_7
#define PROXIMITY4_GPIO_Port GPIOD
#define PROXIMITY4_EXTI_IRQn EXTI9_5_IRQn
#define QEI4B_Pin GPIO_PIN_3
#define QEI4B_GPIO_Port GPIOB
#define QEI2A_Pin GPIO_PIN_4
#define QEI2A_GPIO_Port GPIOB
#define QEI2B_Pin GPIO_PIN_5
#define QEI2B_GPIO_Port GPIOB
#define PC_TX_Pin GPIO_PIN_6
#define PC_TX_GPIO_Port GPIOB
#define PLUSE1_Pin GPIO_PIN_8
#define PLUSE1_GPIO_Port GPIOB
#define PLUSE2_Pin GPIO_PIN_9
#define PLUSE2_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
