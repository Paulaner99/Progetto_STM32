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
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_crc.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {
	NULL_STATE,
	SINGLE_CLICK,
	DOUBLE_CLICK
} btnState_TypeDef;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI_CS_Pin LL_GPIO_PIN_3
#define SPI_CS_GPIO_Port GPIOE
#define BUTTON_BLUE_Pin LL_GPIO_PIN_0
#define BUTTON_BLUE_GPIO_Port GPIOA
#define GREEN_LED_Pin LL_GPIO_PIN_12
#define GREEN_LED_GPIO_Port GPIOD
#define ORANGE_LED_Pin LL_GPIO_PIN_13
#define ORANGE_LED_GPIO_Port GPIOD
#define RED_LED_Pin LL_GPIO_PIN_14
#define RED_LED_GPIO_Port GPIOD
#define BLUE_LED_Pin LL_GPIO_PIN_15
#define BLUE_LED_GPIO_Port GPIOD
#define INT_1_Pin LL_GPIO_PIN_0
#define INT_1_GPIO_Port GPIOE
#define INT_2_Pin LL_GPIO_PIN_1
#define INT_2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define LED_PERIOD_1 500
#define LED_PERIOD_2 250

#define DEBOUNCE_TIMEOUT_MS 30
#define DOUBLECLICK_TIMEOUT_MS 500

#define STREAM_PERIOD_MS 50
#define LED_PERIOD_MS 500

// defining every part of magnified spectrum
#define MAG_SLICE_SIZE 128

#define FIRST_SLICE 0
#define SECOND_SLICE 1
#define THIRD_SLICE 2
#define FOURTH_SLICE 3

// defining thresholds for LED lighting
// two threshold, to reduce flickering
#define H_THRESHOLD 3000.00f
#define L_THRESHOLD 2500.00f

// defining what char we have to receive to start
#define START_CHAR 's'

//defining states
#define OFF 0
#define ON 1

//string size
#define MAX_S 150

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
