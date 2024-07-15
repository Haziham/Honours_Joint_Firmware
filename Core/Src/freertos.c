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
// #include "can_queues.h"
#include "canQueue.h"
#include "can.h"
#include "freckle_protocol.h"
#include "stm32f0xx_hal_tim.h"
#include "tim.h"
#include "adc.h"
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
osThreadId controlSystemHandle;
osThreadId canTransmitHandle;
osMutexId CANTxDataHandle;
osMutexId CANRxDataHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void control_system_task(void const * argument);
void transmit_can_frame_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */


    osThreadDef(controlSystemHandle, control_system_task, osPriorityNormal, 0, 64);
    controlSystemHandle = osThreadCreate(osThread(controlSystemHandle), NULL);
    osThreadDef(canTransmitHandle, transmit_can_frame_task, osPriorityNormal, 0, 64);
    canTransmitHandle = osThreadCreate(osThread(canTransmitHandle), NULL); 

    CANTxDataHandle = xSemaphoreCreateMutex();
    CANRxDataHandle = xSemaphoreCreateMutex(); //

    CAN_InitQueues();
  /* USER CODE END Init */
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

/**
  * @}
  */

/**
  * @}
  */
}

/* USER CODE BEGIN Header_control_system_task */
/**
  * @brief  Function implementing the controlSystemHa thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_control_system_task */
void control_system_task(void const * argument)
{
  /* USER CODE BEGIN control_system_task */
  // __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 50000);
  JointSettings_t jointSettings;
  jointSettings.jointType = JOINT_HIP_YAW;
  jointSettings.legNumber = 0;
  jointSettings.maxAngle = 8.9;
  jointSettings.minAngle = 6.7;
  jointSettings.nodeId = 5;

  CAN_Message_t canMessage;  

  /* Infinite loop */
  for(;;)
  {
    // encodeJointSettingsPacketStructure(&canMessage, &jointSettings);
    // finishFrecklePacket(&canMessage, getJointSettingsMaxDataLength(), getJointSettingsPacketID());
    // xSemaphoreTake(CANTxDataHandle, portMAX_DELAY); // Take the mutex
    // CAN_enqueue_message(&canTxQueue, &canMessage);
    // xSemaphoreGive(CANTxDataHandle); // Give the mutex
    // osDelay(1000);
    uint16_t pwm0;
    uint16_t pwm1;
    if (joint.statusA.enabled)
    {
      switch (joint.commandSettings.mode)
      {
        case CMD_PWM:
          if (joint.command.direction == DIR_FORWARD)
          {

            pwm0 = 65535;
            pwm1 = 65535-joint.command.value;
          }
          else {
            pwm0 = 65535-joint.command.value;
            pwm1 = 65535;
          }
          break;
        case CMD_POSITION:
        case CMD_VELOCITY:
        case CMD_TORQUE:
        default:
          pwm0 = 65535;
          pwm1 = 65535;
          break;
      }
    }
    else {
      pwm0 = 65535;
      pwm1 = 65535;
    }

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwm0);
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm1);
    joint.statusB.current = getCurrent();
    joint.statusB.voltage = getVoltage();
    joint.statusB.externalADC = getExternalVoltage();
    // osDelay(10);
  }
  /* USER CODE END control_system_task */
}

/* USER CODE BEGIN Header_transmit_can_frame_task */
/**
* @brief Function implementing the canTransmitHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transmit_can_frame_task */
void transmit_can_frame_task(void const * argument)
{
  /* USER CODE BEGIN transmit_can_frame_task */
  CAN_Message_t canMessage;
  CAN_TxHeaderTypeDef canHeader;
  uint32_t txMailbox;

  canHeader.IDE = CAN_ID_STD;
  canHeader.RTR = CAN_RTR_DATA;
  uint16_t counter = 0;
  uint8_t temp = 0 ;
  HAL_StatusTypeDef error;

  static uint32_t lastTelemetryTime = 0;
  uint32_t currentTime;
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake(CANTxDataHandle, portMAX_DELAY); // Take the mutex

    // Check if we should send some telemetry. This should be another task, but lacking space.
    currentTime = HAL_GetTick();
    if ((currentTime - lastTelemetryTime) > joint.telemetrySettings.transmitPeriod)
    {
      lastTelemetryTime = currentTime;
      encodeStatusAPacketStructure(&canMessage, &joint.statusA);
      finishFrecklePacket(&canMessage, getStatusAMaxDataLength(), getStatusAPacketID());
      CAN_enqueue_message(&canTxQueue, &canMessage);

      encodeStatusBPacketStructure(&canMessage, &joint.statusB);
      finishFrecklePacket(&canMessage, getStatusBMaxDataLength(), getStatusBPacketID());
      CAN_enqueue_message(&canTxQueue, &canMessage);
    }

    // encodeStatusAPacketStructure(&canMessage, &joint.statusA);
    // finishFrecklePacket(&canMessage, getStatusAMaxDataLength(), getStatusAPacketID());
    // CAN_enqueue_message(&canTxQueue, &canMessage);



    temp = CAN_dequeue_message(&canTxQueue, &canMessage);
    xSemaphoreGive(CANTxDataHandle); // Give the mutex
    if(temp == 0)
    {
      canHeader.StdId = canMessage.id;
      canHeader.DLC = canMessage.len;
      error = HAL_CAN_AddTxMessage(&hcan, &canHeader, canMessage.data, &txMailbox);
      if (error != HAL_OK)
      {
        Error_Handler();
      }
    }
    counter += 1000;
    osDelay(3);
  }
  /* USER CODE END transmit_can_frame_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

