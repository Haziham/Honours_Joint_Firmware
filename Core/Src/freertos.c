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
#include "joint.h"
#include "pid.h"
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

  int16_t previousPosition = 0;
  int16_t deltaPosition = 0;
  uint32_t currentTimeMs = 0;
  uint32_t previousTimeMs = 0;
  int32_t deltaTimeMs = 0;
  uint8_t updateVelocity = 0;
  int32_t dutyCycle = 0;
  uint8_t offset = 0;


  /* Infinite loop */
  for(;;)
  {
    joint.settings.internal.position =  __HAL_TIM_GET_COUNTER(&htim2);
    convert_countToAngle(&joint.statusA.position, joint.settings.internal.position);

    currentTimeMs = HAL_GetTick();
    deltaTimeMs = currentTimeMs - previousTimeMs;
    updateVelocity = 0;
    if (deltaTimeMs > 100)
    {
      deltaPosition = joint.settings.internal.position - previousPosition;
      convert_countToAngle(&joint.statusA.velocity, (deltaPosition*1000)/deltaTimeMs); 
      joint.statusA.moving = deltaPosition != 0;
      joint.statusA.direction = joint.statusA.velocity > 0;
      
      previousPosition = joint.settings.internal.position;
      previousTimeMs = currentTimeMs;
      updateVelocity = 1;
    }


    // joint.statusC.debugValue = joint.command.value;
    if (joint.statusA.enabled)
    {
      switch (joint.settings.command.mode)
      {
        case CMD_DUTY_CYCLE:
          dutyCycle = joint.command.value;
          offset = 1;
          break;
        case CMD_POSITION:
          dutyCycle = PID_calculate(&positionPID, joint.command.value + joint.settings.calibration.angleOffset, joint.statusA.position);
          offset  = 1;
          break;
        case CMD_CALIBRATE:
          if (updateVelocity)
          {
            joint.statusA.calibrating = 1;
            joint_calibrate(&dutyCycle, joint.settings.internal.position, joint.statusA.velocity);
            // joint.statusC.debugValue = pwm;
            offset = 1;
          }
          break;
        case CMD_VELOCITY:
        case CMD_TORQUE:
        default:
          dutyCycle = 0;
          break;
      }
    }
    else {
      dutyCycle = 0;
      offset = 0;
      joint.command.value = 0;
      joint.statusA.calibrating = 0;
      joint.internalFlags.calibrateStep = CALIBRATE_START;
    }

    // joint.statusC.debugValue = joint_isPastStopPoint(joint.statusA.position);
    // joint.statusC.debugValue = joint.statusA.position;
    // joint.statusC.debugValue1 = joint.settings.calibration.minAngle;


    // Check if the joint is past the stop point, prevent it going further.
    AngleStatus angleStatus = checkAngle(joint.statusA.position);
    if ( !joint.statusA.calibrating &&    
         angleStatus != ANGLE_WITHIN_BOUNDS &&
        ((angleStatus == ANGLE_BELOW_MIN && dutyCycle < 0) || (angleStatus == ANGLE_ABOVE_MAX && dutyCycle > 0)))
    {
      dutyCycle = 0;
      offset = 0;
    }


    // joint_setPWMPulseWidth(dutyCycle, offset);
    joint_setDutyCycle(dutyCycle, offset);
    joint.statusB.current = getCurrent();
    joint.statusB.voltage = getVoltage();
    joint.statusB.externalADC = getExternalVoltage();



    osDelay(1);
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

  static uint32_t previousTelemetryTime = 0;
  static uint32_t previousSettingsTime = 0;
  uint32_t currentTime;
  uint32_t deltaTime;
  /* Infinite loop */
  for(;;)
  {
    currentTime = HAL_GetTick();
    if (osMutexWait(CANTxDataHandle, osWaitForever) == osOK)
    {

      // Check if we should send some telemetry. This should be another task, but lacking space.
      deltaTime = currentTime - previousTelemetryTime;
      if (deltaTime > joint.settings.telemetry.transmitPeriod)
      {
        encodeStatusAPacketStructure(&canMessage, &joint.statusA);
        finishFrecklePacket(&canMessage, getStatusAMaxDataLength(), getStatusAPacketID());
        CAN_enqueue_message(&canTxQueue, &canMessage);

        encodeStatusBPacketStructure(&canMessage, &joint.statusB);
        finishFrecklePacket(&canMessage, getStatusBMaxDataLength(), getStatusBPacketID());
        CAN_enqueue_message(&canTxQueue, &canMessage);

        if (joint.statusA.debug)
        {
          encodeStatusCPacketStructure(&canMessage, &joint.statusC);
          finishFrecklePacket(&canMessage, getStatusCMaxDataLength(), getStatusCPacketID());
          CAN_enqueue_message(&canTxQueue, &canMessage);
        }
        previousTelemetryTime = currentTime;
      }


      if(CAN_dequeue_message(&canTxQueue, &canMessage) == SUCCESS_L)
      {
        CAN_SendMessage(&canMessage);
      }
    }
    osMutexRelease(CANTxDataHandle); // Give the mutex
    // Every 10 seconds save settings is not enabled and settings changed. 
    deltaTime = currentTime - previousSettingsTime;
    if (deltaTime > 1000 && !joint.statusA.enabled && joint.internalFlags.saveSettingsFlag)
    {
      // Disable interrupts
      __disable_irq();
      save_settings();
      __enable_irq();
      previousSettingsTime = currentTime;
      joint.internalFlags.saveSettingsFlag = 0;
    }

    osDelay(3);
  }
  /* USER CODE END transmit_can_frame_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

