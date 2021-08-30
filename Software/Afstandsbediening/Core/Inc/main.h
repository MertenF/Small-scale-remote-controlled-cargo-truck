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
#include "stm32f3xx_hal.h"

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
#define joy2_ver_Pin GPIO_PIN_0
#define joy2_ver_GPIO_Port GPIOC
#define joy2_hor_Pin GPIO_PIN_1
#define joy2_hor_GPIO_Port GPIOC
#define joy2_knop_Pin GPIO_PIN_2
#define joy2_knop_GPIO_Port GPIOC
#define piezo_Pin GPIO_PIN_3
#define piezo_GPIO_Port GPIOC
#define joy1_knop_Pin GPIO_PIN_0
#define joy1_knop_GPIO_Port GPIOA
#define joy2_horA1_Pin GPIO_PIN_1
#define joy2_horA1_GPIO_Port GPIOA
#define joy1_ver_Pin GPIO_PIN_2
#define joy1_ver_GPIO_Port GPIOA
#define nrf_CE_Pin GPIO_PIN_3
#define nrf_CE_GPIO_Port GPIOA
#define csn_Pin GPIO_PIN_4
#define csn_GPIO_Port GPIOA
#define rotary1_Pin GPIO_PIN_0
#define rotary1_GPIO_Port GPIOB
#define rotary2_Pin GPIO_PIN_1
#define rotary2_GPIO_Port GPIOB
#define rotary_switch_Pin GPIO_PIN_2
#define rotary_switch_GPIO_Port GPIOB
#define SPI1_IRQ_Pin GPIO_PIN_8
#define SPI1_IRQ_GPIO_Port GPIOA
#define button_left_Pin GPIO_PIN_5
#define button_left_GPIO_Port GPIOB
#define button_up_Pin GPIO_PIN_6
#define button_up_GPIO_Port GPIOB
#define button_down_Pin GPIO_PIN_7
#define button_down_GPIO_Port GPIOB
#define button_pinker_links_Pin GPIO_PIN_8
#define button_pinker_links_GPIO_Port GPIOB
#define button_pinker_rechts_Pin GPIO_PIN_9
#define button_pinker_rechts_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
