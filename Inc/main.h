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
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"

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
#define RMotorPWM_Pin LL_GPIO_PIN_5
#define RMotorPWM_GPIO_Port GPIOE
#define LMotorPWM_Pin LL_GPIO_PIN_6
#define LMotorPWM_GPIO_Port GPIOE
#define Servo_Pin LL_GPIO_PIN_7
#define Servo_GPIO_Port GPIOF
#define Trigger_Pin LL_GPIO_PIN_8
#define Trigger_GPIO_Port GPIOF
#define FLPres_Pin LL_GPIO_PIN_0
#define FLPres_GPIO_Port GPIOA
#define FRPres_Pin LL_GPIO_PIN_1
#define FRPres_GPIO_Port GPIOA
#define MLPres_Pin LL_GPIO_PIN_2
#define MLPres_GPIO_Port GPIOA
#define MRPres_Pin LL_GPIO_PIN_3
#define MRPres_GPIO_Port GPIOA
#define BLPres_Pin LL_GPIO_PIN_4
#define BLPres_GPIO_Port GPIOA
#define BRPres_Pin LL_GPIO_PIN_5
#define BRPres_GPIO_Port GPIOA
#define MPres_Pin LL_GPIO_PIN_6
#define MPres_GPIO_Port GPIOA
#define FLUltra_Pin LL_GPIO_PIN_9
#define FLUltra_GPIO_Port GPIOE
#define FRUltra_Pin LL_GPIO_PIN_13
#define FRUltra_GPIO_Port GPIOE
#define BRUltra_Pin LL_GPIO_PIN_10
#define BRUltra_GPIO_Port GPIOB
#define CS_Pin LL_GPIO_PIN_12
#define CS_GPIO_Port GPIOB
#define LMotorEncB_Pin LL_GPIO_PIN_12
#define LMotorEncB_GPIO_Port GPIOD
#define LMotorEncA_Pin LL_GPIO_PIN_13
#define LMotorEncA_GPIO_Port GPIOD
#define LMotorINB_Pin LL_GPIO_PIN_5
#define LMotorINB_GPIO_Port GPIOG
#define LMotorINA_Pin LL_GPIO_PIN_6
#define LMotorINA_GPIO_Port GPIOG
#define RMotorINB_Pin LL_GPIO_PIN_7
#define RMotorINB_GPIO_Port GPIOG
#define RMotorINA_Pin LL_GPIO_PIN_8
#define RMotorINA_GPIO_Port GPIOG
#define RMotorEncA_Pin LL_GPIO_PIN_6
#define RMotorEncA_GPIO_Port GPIOC
#define RMotorEncB_Pin LL_GPIO_PIN_7
#define RMotorEncB_GPIO_Port GPIOC
#define INT_Pin LL_GPIO_PIN_8
#define INT_GPIO_Port GPIOC
#define RST_Pin LL_GPIO_PIN_9
#define RST_GPIO_Port GPIOC
#define PS0_WAKE_Pin LL_GPIO_PIN_8
#define PS0_WAKE_GPIO_Port GPIOA
#define BLUltra_Pin LL_GPIO_PIN_15
#define BLUltra_GPIO_Port GPIOA
#define BRValv_Pin LL_GPIO_PIN_10
#define BRValv_GPIO_Port GPIOC
#define BLValv_Pin LL_GPIO_PIN_11
#define BLValv_GPIO_Port GPIOC
#define MRValv_Pin LL_GPIO_PIN_12
#define MRValv_GPIO_Port GPIOC
#define MLValv_Pin LL_GPIO_PIN_0
#define MLValv_GPIO_Port GPIOD
#define FRValv_Pin LL_GPIO_PIN_1
#define FRValv_GPIO_Port GPIOD
#define FLValv_Pin LL_GPIO_PIN_2
#define FLValv_GPIO_Port GPIOD
#define MValv_Pin LL_GPIO_PIN_4
#define MValv_GPIO_Port GPIOD
#define ModeControl_Pin LL_GPIO_PIN_5
#define ModeControl_GPIO_Port GPIOD
#define RProx_Pin LL_GPIO_PIN_13
#define RProx_GPIO_Port GPIOG
#define LProx_Pin LL_GPIO_PIN_14
#define LProx_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
