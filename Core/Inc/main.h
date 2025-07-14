/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN_LED2_Pin GPIO_PIN_13
#define CAN_LED2_GPIO_Port GPIOC
#define ENC1_A_Pin GPIO_PIN_14
#define ENC1_A_GPIO_Port GPIOC
#define ENC1_B_Pin GPIO_PIN_15
#define ENC1_B_GPIO_Port GPIOC
#define ENC3_A_Pin GPIO_PIN_0
#define ENC3_A_GPIO_Port GPIOC
#define ENC3_B_Pin GPIO_PIN_1
#define ENC3_B_GPIO_Port GPIOC
#define ENC4_A_Pin GPIO_PIN_2
#define ENC4_A_GPIO_Port GPIOC
#define ENC4_B_Pin GPIO_PIN_3
#define ENC4_B_GPIO_Port GPIOC
#define DIP1_Pin GPIO_PIN_0
#define DIP1_GPIO_Port GPIOA
#define DIP2_Pin GPIO_PIN_1
#define DIP2_GPIO_Port GPIOA
#define ENC2_A_Pin GPIO_PIN_2
#define ENC2_A_GPIO_Port GPIOA
#define ENC2_B_Pin GPIO_PIN_3
#define ENC2_B_GPIO_Port GPIOA
#define DIP3_Pin GPIO_PIN_4
#define DIP3_GPIO_Port GPIOA
#define DIP4_Pin GPIO_PIN_5
#define DIP4_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOC
#define M3_DIR_Pin GPIO_PIN_0
#define M3_DIR_GPIO_Port GPIOB
#define M4_DIR_Pin GPIO_PIN_1
#define M4_DIR_GPIO_Port GPIOB
#define M2_DIR_Pin GPIO_PIN_2
#define M2_DIR_GPIO_Port GPIOB
#define M2_DIRB10_Pin GPIO_PIN_10
#define M2_DIRB10_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_12
#define SW5_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_13
#define SW4_GPIO_Port GPIOB
#define BZ_Pin GPIO_PIN_14
#define BZ_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_15
#define SW3_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_6
#define SW2_GPIO_Port GPIOC
#define M1_PWM_Pin GPIO_PIN_7
#define M1_PWM_GPIO_Port GPIOC
#define M2_PWM_Pin GPIO_PIN_8
#define M2_PWM_GPIO_Port GPIOC
#define M3_PWM_Pin GPIO_PIN_9
#define M3_PWM_GPIO_Port GPIOA
#define M4_PWM_Pin GPIO_PIN_10
#define M4_PWM_GPIO_Port GPIOA
#define CAN_LED1_Pin GPIO_PIN_7
#define CAN_LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2

#define Button_UP 0
#define Button_Down 1
#define Button_Left 2
#define Button_Right 3
#define Button_A 4
#define Button_B 5
#define Button_X 6
#define Button_Y 7

#define Button_L1 8
#define Button_R1 9
#define Button_L2 10
#define Button_R2 11
#define Button_LS 12
#define Button_RS 13
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
