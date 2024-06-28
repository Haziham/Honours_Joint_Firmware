/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
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
/* Definitions for controlSystem */
osThreadId_t controlSystemHandle;
uint32_t controlSystemBuffer[ 128 ];
osStaticThreadDef_t controlSystemControlBlock;
const osThreadAttr_t controlSystem_attributes = {
  .name = "controlSystem",
  .cb_mem = &controlSystemControlBlock,
  .cb_size = sizeof(controlSystemControlBlock),
  .stack_mem = &controlSystemBuffer[0],
  .stack_size = sizeof(controlSystemBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for canTransmit */
osThreadId_t canTransmitHandle;
uint32_t canTransmitBuffer[ 128 ];
osStaticThreadDef_t canTransmitControlBlock;
const osThreadAttr_t canTransmit_attributes = {
  .name = "canTransmit",
  .cb_mem = &canTransmitControlBlock,
  .cb_size = sizeof(canTransmitControlBlock),
  .stack_mem = &canTransmitBuffer[0],
  .stack_size = sizeof(canTransmitBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CANTxData */
osMutexId_t CANTxDataHandle;
osStaticMutexDef_t CANTxDataControlBlock;
const osMutexAttr_t CANTxData_attributes = {
  .name = "CANTxData",
  .cb_mem = &CANTxDataControlBlock,
  .cb_size = sizeof(CANTxDataControlBlock),
};
/* Definitions for CANRxData */
osMutexId_t CANRxDataHandle;
osStaticMutexDef_t CANRxDataControlBlock;
const osMutexAttr_t CANRxData_attributes = {
  .name = "CANRxData",
  .cb_mem = &CANRxDataControlBlock,
  .cb_size = sizeof(CANRxDataControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void startControlSystem(void *argument);
void transmit_can_frame(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of CANTxData */
  CANTxDataHandle = osMutexNew(&CANTxData_attributes);

  /* creation of CANRxData */
  CANRxDataHandle = osMutexNew(&CANRxData_attributes);

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
  /* creation of controlSystem */
  controlSystemHandle = osThreadNew(startControlSystem, NULL, &controlSystem_attributes);

  /* creation of canTransmit */
  canTransmitHandle = osThreadNew(transmit_can_frame, NULL, &canTransmit_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_startControlSystem */
/**
  * @brief  Function implementing the controlSystem thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startControlSystem */
void startControlSystem(void *argument)
{
  /* USER CODE BEGIN startControlSystem */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END startControlSystem */
}

/* USER CODE BEGIN Header_transmit_can_frame */
/**
* @brief Function implementing the canTransmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transmit_can_frame */
void transmit_can_frame(void *argument)
{
  /* USER CODE BEGIN transmit_can_frame */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END transmit_can_frame */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

