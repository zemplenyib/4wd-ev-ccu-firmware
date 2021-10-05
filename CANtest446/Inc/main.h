/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

#define	ON					(1)				// Constant for On state
#define	OFF					(0)				// Constant for Off state

#define	NUL_C				(0x00)			// Character constant
#define	CR_C				(0x0D)			// Character constant

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/**
  * @brief  Generating USART Communication Error Message.
  *
  * @retval None
  */
void COMerrorSignal(void);

  /**
    * @brief  Generating Command Error Message.
    *
    * @retval None
    */
  void CMDerrorSignal(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_5
#define GREEN_LED_GPIO_Port GPIOA
#define VPOT1_ADC1_14_Pin GPIO_PIN_4
#define VPOT1_ADC1_14_GPIO_Port GPIOC
#define VPOT2_ADC1_15_Pin GPIO_PIN_5
#define VPOT2_ADC1_15_GPIO_Port GPIOC
#define USR_LED2_Pin GPIO_PIN_1
#define USR_LED2_GPIO_Port GPIOB
#define USR_LED1_Pin GPIO_PIN_2
#define USR_LED1_GPIO_Port GPIOB
#define USR_SW1_Pin GPIO_PIN_12
#define USR_SW1_GPIO_Port GPIOB
#define USR_SW1_EXTI_IRQn EXTI15_10_IRQn
#define USR_SW4_Pin GPIO_PIN_13
#define USR_SW4_GPIO_Port GPIOB
#define USR_SW4_EXTI_IRQn EXTI15_10_IRQn
#define USR_SW3_Pin GPIO_PIN_14
#define USR_SW3_GPIO_Port GPIOB
#define USR_SW3_EXTI_IRQn EXTI15_10_IRQn
#define USR_SW2_Pin GPIO_PIN_15
#define USR_SW2_GPIO_Port GPIOB
#define USR_SW2_EXTI_IRQn EXTI15_10_IRQn
#define USR_LED5_Pin GPIO_PIN_6
#define USR_LED5_GPIO_Port GPIOC
#define USR_LED3_Pin GPIO_PIN_7
#define USR_LED3_GPIO_Port GPIOC
#define USR_LED6_Pin GPIO_PIN_8
#define USR_LED6_GPIO_Port GPIOC
#define USR_LED4_Pin GPIO_PIN_9
#define USR_LED4_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define RED_3CLED_TIM3_1_Pin GPIO_PIN_4
#define RED_3CLED_TIM3_1_GPIO_Port GPIOB
#define GREEN_3CLED_TIM3_2_Pin GPIO_PIN_5
#define GREEN_3CLED_TIM3_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
