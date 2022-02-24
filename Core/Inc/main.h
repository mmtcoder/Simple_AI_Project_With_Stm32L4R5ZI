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
#include "stm32l4xx_hal.h"

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
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Cam_PCLK_Pin GPIO_PIN_7
#define Cam_PCLK_GPIO_Port GPIOF
#define Cam_HREF_Pin GPIO_PIN_8
#define Cam_HREF_GPIO_Port GPIOF
#define Cam_Vsyn_EXTI_Pin GPIO_PIN_9
#define Cam_Vsyn_EXTI_GPIO_Port GPIOF
#define Cam_Vsyn_EXTI_EXTI_IRQn EXTI9_5_IRQn
#define CAM_RESET_Pin GPIO_PIN_2
#define CAM_RESET_GPIO_Port GPIOC
#define CAM_PWDN_Pin GPIO_PIN_3
#define CAM_PWDN_GPIO_Port GPIOC
#define LED_ACTIVATE_Pin GPIO_PIN_5
#define LED_ACTIVATE_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_OverCurrent_Pin GPIO_PIN_6
#define USB_OverCurrent_GPIO_Port GPIOG
#define STLINK_TX_Pin GPIO_PIN_7
#define STLINK_TX_GPIO_Port GPIOG
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOG
#define SPI_SD_NSS_Pin GPIO_PIN_6
#define SPI_SD_NSS_GPIO_Port GPIOC
#define LED_CS_Pin GPIO_PIN_8
#define LED_CS_GPIO_Port GPIOC
#define LED_DC_Pin GPIO_PIN_9
#define LED_DC_GPIO_Port GPIOC
#define LED_RST_Pin GPIO_PIN_10
#define LED_RST_GPIO_Port GPIOC
#define Cam_D0_Pin GPIO_PIN_0
#define Cam_D0_GPIO_Port GPIOD
#define Cam_D1_Pin GPIO_PIN_1
#define Cam_D1_GPIO_Port GPIOD
#define Cam_D2_Pin GPIO_PIN_2
#define Cam_D2_GPIO_Port GPIOD
#define Cam_D3_Pin GPIO_PIN_3
#define Cam_D3_GPIO_Port GPIOD
#define Cam_D4_Pin GPIO_PIN_4
#define Cam_D4_GPIO_Port GPIOD
#define Cam_D5_Pin GPIO_PIN_5
#define Cam_D5_GPIO_Port GPIOD
#define Cam_D6_Pin GPIO_PIN_6
#define Cam_D6_GPIO_Port GPIOD
#define Cam_D7_Pin GPIO_PIN_7
#define Cam_D7_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define User_LD2_Blue_Pin GPIO_PIN_7
#define User_LD2_Blue_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
