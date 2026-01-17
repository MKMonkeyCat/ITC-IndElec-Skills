/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DATA_DP_Pin GPIO_PIN_0
#define DATA_DP_GPIO_Port GPIOC
#define DATA_G_Pin GPIO_PIN_1
#define DATA_G_GPIO_Port GPIOC
#define DATA_F_Pin GPIO_PIN_2
#define DATA_F_GPIO_Port GPIOC
#define DATA_E_Pin GPIO_PIN_3
#define DATA_E_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define DATA_D_Pin GPIO_PIN_4
#define DATA_D_GPIO_Port GPIOC
#define DATA_C_Pin GPIO_PIN_5
#define DATA_C_GPIO_Port GPIOC
#define DS1_Pin GPIO_PIN_0
#define DS1_GPIO_Port GPIOB
#define DS2_Pin GPIO_PIN_1
#define DS2_GPIO_Port GPIOB
#define DS3_Pin GPIO_PIN_2
#define DS3_GPIO_Port GPIOB
#define DATA_B_Pin GPIO_PIN_6
#define DATA_B_GPIO_Port GPIOC
#define DATA_A_Pin GPIO_PIN_7
#define DATA_A_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_9
#define SW2_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_10
#define SW3_GPIO_Port GPIOC
#define SW4_Pin GPIO_PIN_11
#define SW4_GPIO_Port GPIOC
#define SW5_Pin GPIO_PIN_12
#define SW5_GPIO_Port GPIOC
#define DS4_Pin GPIO_PIN_3
#define DS4_GPIO_Port GPIOB
#define DS5_Pin GPIO_PIN_4
#define DS5_GPIO_Port GPIOB
#define DS6_Pin GPIO_PIN_5
#define DS6_GPIO_Port GPIOB
#define MORSE_SIGNAL_Pin GPIO_PIN_6
#define MORSE_SIGNAL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
