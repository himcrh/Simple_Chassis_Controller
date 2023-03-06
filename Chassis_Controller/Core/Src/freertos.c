/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Chassis_Move */
osThreadId_t Chassis_MoveHandle;
const osThreadAttr_t Chassis_Move_attributes = {
  .name = "Chassis_Move",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Line_Update */
osThreadId_t Line_UpdateHandle;
const osThreadAttr_t Line_Update_attributes = {
  .name = "Line_Update",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Arm_Move_Basic */
osThreadId_t Arm_Move_BasicHandle;
const osThreadAttr_t Arm_Move_Basic_attributes = {
  .name = "Arm_Move_Basic",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Arm_Move_Contro */
osThreadId_t Arm_Move_ControHandle;
const osThreadAttr_t Arm_Move_Contro_attributes = {
  .name = "Arm_Move_Contro",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Chassis_Run(void *argument);
void Line_Refresh(void *argument);
void Arm_Move_Base(void *argument);
void Arm_Move_Ctrl(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Chassis_Move */
  Chassis_MoveHandle = osThreadNew(Chassis_Run, NULL, &Chassis_Move_attributes);

  /* creation of Line_Update */
  Line_UpdateHandle = osThreadNew(Line_Refresh, NULL, &Line_Update_attributes);

  /* creation of Arm_Move_Basic */
  Arm_Move_BasicHandle = osThreadNew(Arm_Move_Base, NULL, &Arm_Move_Basic_attributes);

  /* creation of Arm_Move_Contro */
  Arm_Move_ControHandle = osThreadNew(Arm_Move_Ctrl, NULL, &Arm_Move_Contro_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Chassis_Run */
/**
  * @brief  Function implementing the Chassis_Move thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Chassis_Run */
__weak void Chassis_Run(void *argument)
{
  /* USER CODE BEGIN Chassis_Run */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Run */
}

/* USER CODE BEGIN Header_Line_Refresh */
/**
* @brief Function implementing the Line_Update thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Line_Refresh */
__weak void Line_Refresh(void *argument)
{
  /* USER CODE BEGIN Line_Refresh */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Line_Refresh */
}

/* USER CODE BEGIN Header_Arm_Move_Base */
/**
* @brief Function implementing the Arm_Move_Basic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Arm_Move_Base */
__weak void Arm_Move_Base(void *argument)
{
  /* USER CODE BEGIN Arm_Move_Base */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Arm_Move_Base */
}

/* USER CODE BEGIN Header_Arm_Move_Ctrl */
/**
* @brief Function implementing the Arm_Move_Contro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Arm_Move_Ctrl */
__weak void Arm_Move_Ctrl(void *argument)
{
  /* USER CODE BEGIN Arm_Move_Ctrl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Arm_Move_Ctrl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

