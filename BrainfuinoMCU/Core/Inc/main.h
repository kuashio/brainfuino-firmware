/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOE
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOE
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOE
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOE
#define A6_Pin GPIO_PIN_6
#define A6_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOF
#define AIN0_Pin GPIO_PIN_0
#define AIN0_GPIO_Port GPIOC
#define BF_INSTRB_Pin GPIO_PIN_1
#define BF_INSTRB_GPIO_Port GPIOC
#define BF_INSTRB_EXTI_IRQn EXTI0_1_IRQn
#define BRD_INSTRB_Pin GPIO_PIN_2
#define BRD_INSTRB_GPIO_Port GPIOC
#define BF_OUTSTRB_Pin GPIO_PIN_3
#define BF_OUTSTRB_GPIO_Port GPIOC
#define BF_OUTSTRB_EXTI_IRQn EXTI2_3_IRQn
#define BF_IN0_Pin GPIO_PIN_0
#define BF_IN0_GPIO_Port GPIOA
#define BF_IN1_Pin GPIO_PIN_1
#define BF_IN1_GPIO_Port GPIOA
#define BF_IN2_Pin GPIO_PIN_2
#define BF_IN2_GPIO_Port GPIOA
#define BF_IN3_Pin GPIO_PIN_3
#define BF_IN3_GPIO_Port GPIOA
#define BF_IN4_Pin GPIO_PIN_4
#define BF_IN4_GPIO_Port GPIOA
#define BF_IN5_Pin GPIO_PIN_5
#define BF_IN5_GPIO_Port GPIOA
#define BF_IN6_Pin GPIO_PIN_6
#define BF_IN6_GPIO_Port GPIOA
#define BF_IN7_Pin GPIO_PIN_7
#define BF_IN7_GPIO_Port GPIOA
#define BRD_RST_Pin GPIO_PIN_4
#define BRD_RST_GPIO_Port GPIOC
#define AIEN_Pin GPIO_PIN_5
#define AIEN_GPIO_Port GPIOC
#define BF_OUT0_Pin GPIO_PIN_0
#define BF_OUT0_GPIO_Port GPIOB
#define BF_OUT1_Pin GPIO_PIN_1
#define BF_OUT1_GPIO_Port GPIOB
#define BF_OUT2_Pin GPIO_PIN_2
#define BF_OUT2_GPIO_Port GPIOB
#define A7_Pin GPIO_PIN_7
#define A7_GPIO_Port GPIOE
#define A8_Pin GPIO_PIN_8
#define A8_GPIO_Port GPIOE
#define A9_Pin GPIO_PIN_9
#define A9_GPIO_Port GPIOE
#define A10_Pin GPIO_PIN_10
#define A10_GPIO_Port GPIOE
#define A11_Pin GPIO_PIN_11
#define A11_GPIO_Port GPIOE
#define A12_Pin GPIO_PIN_12
#define A12_GPIO_Port GPIOE
#define A13_Pin GPIO_PIN_13
#define A13_GPIO_Port GPIOE
#define A14_Pin GPIO_PIN_14
#define A14_GPIO_Port GPIOE
#define A15_Pin GPIO_PIN_15
#define A15_GPIO_Port GPIOE
#define D2_Pin GPIO_PIN_10
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_11
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_15
#define D7_GPIO_Port GPIOB
#define BRD_INCMG_Pin GPIO_PIN_6
#define BRD_INCMG_GPIO_Port GPIOC
#define BF_INCMG_Pin GPIO_PIN_7
#define BF_INCMG_GPIO_Port GPIOC
#define A16_Pin GPIO_PIN_8
#define A16_GPIO_Port GPIOC
#define A17_Pin GPIO_PIN_9
#define A17_GPIO_Port GPIOC
#define BF_CLK_Pin GPIO_PIN_8
#define BF_CLK_GPIO_Port GPIOA
#define OE_Pin GPIO_PIN_10
#define OE_GPIO_Port GPIOC
#define WE_Pin GPIO_PIN_11
#define WE_GPIO_Port GPIOC
#define BF_RST_Pin GPIO_PIN_12
#define BF_RST_GPIO_Port GPIOC
#define IN0_Pin GPIO_PIN_0
#define IN0_GPIO_Port GPIOD
#define IN1_Pin GPIO_PIN_1
#define IN1_GPIO_Port GPIOD
#define IN2_Pin GPIO_PIN_2
#define IN2_GPIO_Port GPIOD
#define IN3_Pin GPIO_PIN_3
#define IN3_GPIO_Port GPIOD
#define IN4_Pin GPIO_PIN_4
#define IN4_GPIO_Port GPIOD
#define IN5_Pin GPIO_PIN_5
#define IN5_GPIO_Port GPIOD
#define IN6_Pin GPIO_PIN_6
#define IN6_GPIO_Port GPIOD
#define IN7_Pin GPIO_PIN_7
#define IN7_GPIO_Port GPIOD
#define BF_OUT3_Pin GPIO_PIN_3
#define BF_OUT3_GPIO_Port GPIOB
#define BF_OUT4_Pin GPIO_PIN_4
#define BF_OUT4_GPIO_Port GPIOB
#define BF_OUT5_Pin GPIO_PIN_5
#define BF_OUT5_GPIO_Port GPIOB
#define BF_OUT6_Pin GPIO_PIN_6
#define BF_OUT6_GPIO_Port GPIOB
#define BF_OUT7_Pin GPIO_PIN_7
#define BF_OUT7_GPIO_Port GPIOB
#define D0_Pin GPIO_PIN_8
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_9
#define D1_GPIO_Port GPIOB
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOE
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
