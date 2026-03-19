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
#include "stm32g4xx_hal.h"

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
#define SHA_C0_2IN6_Pin GPIO_PIN_0
#define SHA_C0_2IN6_GPIO_Port GPIOC
#define SHB_C1_2IN7_Pin GPIO_PIN_1
#define SHB_C1_2IN7_GPIO_Port GPIOC
#define SHC_C2_2IN8_Pin GPIO_PIN_2
#define SHC_C2_2IN8_GPIO_Port GPIOC
#define IA_A0_1IN1_Pin GPIO_PIN_0
#define IA_A0_1IN1_GPIO_Port GPIOA
#define IB_A1_1IN2_Pin GPIO_PIN_1
#define IB_A1_1IN2_GPIO_Port GPIOA
#define IC_A2_1IN3_Pin GPIO_PIN_2
#define IC_A2_1IN3_GPIO_Port GPIOA
#define IBUS_A3_1IN4_Pin GPIO_PIN_3
#define IBUS_A3_1IN4_GPIO_Port GPIOA
#define HALL_A_Pin GPIO_PIN_6
#define HALL_A_GPIO_Port GPIOA
#define HALL_B_Pin GPIO_PIN_7
#define HALL_B_GPIO_Port GPIOA
#define POT_C4_2IN5_Pin GPIO_PIN_4
#define POT_C4_2IN5_GPIO_Port GPIOC
#define VBUS_C5_2IN11_Pin GPIO_PIN_5
#define VBUS_C5_2IN11_GPIO_Port GPIOC
#define HALL_C_Pin GPIO_PIN_0
#define HALL_C_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_6
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_7
#define KEY2_GPIO_Port GPIOC
#define KEY3_Pin GPIO_PIN_8
#define KEY3_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_9
#define KEY4_GPIO_Port GPIOC
#define LCD_RES_Pin GPIO_PIN_15
#define LCD_RES_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_11
#define LCD_DC_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_6
#define LCD_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
