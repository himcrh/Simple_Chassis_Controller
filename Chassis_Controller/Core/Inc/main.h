/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define Rt_IO_L4_Pin GPIO_PIN_2
#define Rt_IO_L4_GPIO_Port GPIOE
#define Lt_IO_L4_Pin GPIO_PIN_3
#define Lt_IO_L4_GPIO_Port GPIOE
#define Rt_IO_L5_Pin GPIO_PIN_4
#define Rt_IO_L5_GPIO_Port GPIOE
#define Lt_IO_L5_Pin GPIO_PIN_5
#define Lt_IO_L5_GPIO_Port GPIOE
#define Rt_IO_L6_Pin GPIO_PIN_6
#define Rt_IO_L6_GPIO_Port GPIOE
#define Lt_IO_L6_Pin GPIO_PIN_13
#define Lt_IO_L6_GPIO_Port GPIOC
#define Rt_IO_L7_Pin GPIO_PIN_14
#define Rt_IO_L7_GPIO_Port GPIOC
#define Lt_IO_L7_Pin GPIO_PIN_15
#define Lt_IO_L7_GPIO_Port GPIOC
#define Fr_IO_L8_Pin GPIO_PIN_12
#define Fr_IO_L8_GPIO_Port GPIOB
#define Bk_IO_L8_Pin GPIO_PIN_13
#define Bk_IO_L8_GPIO_Port GPIOB
#define Fr_IO_L7_Pin GPIO_PIN_14
#define Fr_IO_L7_GPIO_Port GPIOB
#define Bk_IO_L7_Pin GPIO_PIN_15
#define Bk_IO_L7_GPIO_Port GPIOB
#define Bk_IO_L6_Pin GPIO_PIN_10
#define Bk_IO_L6_GPIO_Port GPIOD
#define Fr_IO_L6_Pin GPIO_PIN_11
#define Fr_IO_L6_GPIO_Port GPIOD
#define Bk_IO_L5_Pin GPIO_PIN_12
#define Bk_IO_L5_GPIO_Port GPIOD
#define Fr_IO_L5_Pin GPIO_PIN_13
#define Fr_IO_L5_GPIO_Port GPIOD
#define Bk_IO_L4_Pin GPIO_PIN_14
#define Bk_IO_L4_GPIO_Port GPIOD
#define Bk_IO_L2_Pin GPIO_PIN_15
#define Bk_IO_L2_GPIO_Port GPIOD
#define Fr_IO_L4_Pin GPIO_PIN_2
#define Fr_IO_L4_GPIO_Port GPIOG
#define Fr_IO_L2_Pin GPIO_PIN_3
#define Fr_IO_L2_GPIO_Port GPIOG
#define Bk_IO_L3_Pin GPIO_PIN_4
#define Bk_IO_L3_GPIO_Port GPIOG
#define Bk_IO_L1_Pin GPIO_PIN_5
#define Bk_IO_L1_GPIO_Port GPIOG
#define Fr_IO_L3_Pin GPIO_PIN_6
#define Fr_IO_L3_GPIO_Port GPIOG
#define Fr_IO_L1_Pin GPIO_PIN_7
#define Fr_IO_L1_GPIO_Port GPIOG
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOG
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOG
#define Lt_IO_L3_Pin GPIO_PIN_6
#define Lt_IO_L3_GPIO_Port GPIOB
#define Rt_IO_L3_Pin GPIO_PIN_7
#define Rt_IO_L3_GPIO_Port GPIOB
#define Rt_IO_L1_Pin GPIO_PIN_8
#define Rt_IO_L1_GPIO_Port GPIOB
#define Lt_IO_L1_Pin GPIO_PIN_9
#define Lt_IO_L1_GPIO_Port GPIOB
#define Rt_IO_L2_Pin GPIO_PIN_0
#define Rt_IO_L2_GPIO_Port GPIOE
#define Lt_IO_L2_Pin GPIO_PIN_1
#define Lt_IO_L2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
