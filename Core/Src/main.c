/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

PID_t positionPID;

void control_system_task()
{
  int16_t currentPosition = 0;
  int16_t previousPosition = 0;
  int16_t deltaPosition = 0;
  uint32_t currentTimeMs = 0;
  static uint32_t previousTimeMs = 0;
  int32_t deltaTimeMs = 0;

  currentTimeMs = HAL_GetTick();
  deltaTimeMs = currentTimeMs - previousTimeMs;

  if (deltaTimeMs < 1)
  {
    return;
  }

  currentPosition = __HAL_TIM_GET_COUNTER(&htim2);
  deltaPosition = currentPosition - previousPosition;


  convert_countToAngle(&joint.statusA.position, currentPosition);
  convert_countToAngle(&joint.statusA.velocity, (deltaPosition*1000)/deltaTimeMs); 

  joint.statusA.moving = deltaPosition != 0;
  joint.statusA.direction = joint.statusA.velocity > 0;

  uint32_t pwm = 0;
  uint8_t offset = 0;
  if (joint.statusA.enabled)
  {
    switch (joint.commandSettings.mode)
    {
      case CMD_PWM:
        pwm = joint.command.value;
        offset = 1;
        break;
      case CMD_POSITION:
        pwm = PID_calculate(&positionPID, joint.command.value, joint.statusA.position);
        // joint.statusC.debugValue = joint.command.value;
        joint.statusC.debugValue = pwm;
        offset  = 1;
        // joint.statusC.debugValue = joint.command.value - joint.statusA.position;
        break;
      case CMD_VELOCITY:
      case CMD_TORQUE:
      default:
        pwm = 0;
        break;
    }
  }
  else {
    pwm = 0;
    offset = 0;
    joint.command.value = 0;
  }


  set_motorPWM(pwm, offset);
  joint.statusB.current = getCurrent();
  joint.statusB.voltage = getVoltage();
  joint.statusB.externalADC = getExternalVoltage();
  // joint.statusC.debugValue = deltaTimeMs;


  previousPosition = currentPosition;
  previousTimeMs = currentTimeMs;
}

void transmit_can_frame_task()
{
  CAN_Message_t canMessage;

  uint32_t currentTimeMs = 0;
  static uint32_t previousTimeMs = 0;
  int32_t deltaTimeMs = 0;

  currentTimeMs = HAL_GetTick();
  deltaTimeMs = currentTimeMs - previousTimeMs;

  if (deltaTimeMs < 3)
  {
    return;
  }

  if(CAN_dequeue_message(&canTxQueue, &canMessage))
  {
    CAN_SendMessage(&canMessage);
  }

  previousTimeMs = currentTimeMs;
}

void transmit_telemetry_task()
{
  uint32_t currentTimeMs = 0;
  static uint32_t previousTimeMs = 0;
  int32_t deltaTimeMs = 0;
  CAN_Message_t canMessage;

  currentTimeMs = HAL_GetTick();
  deltaTimeMs = currentTimeMs - previousTimeMs;

  if (deltaTimeMs < 3)
  {
    return;
  }

  encodeStatusAPacketStructure(&canMessage, &joint.statusA);
  finishFrecklePacket(&canMessage, getStatusAMaxDataLength(), getStatusAPacketID());
  CAN_enqueue_message(&canTxQueue, &canMessage);

  encodeStatusBPacketStructure(&canMessage, &joint.statusB);
  finishFrecklePacket(&canMessage, getStatusBMaxDataLength(), getStatusBPacketID());
  CAN_enqueue_message(&canTxQueue, &canMessage);

  encodeStatusCPacketStructure(&canMessage, &joint.statusC);
  finishFrecklePacket(&canMessage, getStatusCMaxDataLength(), getStatusCPacketID());
  CAN_enqueue_message(&canTxQueue, &canMessage);

  previousTimeMs = currentTimeMs;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_CAN_Start(&hcan);


  PID_init(&positionPID, 650, 500, 120, 0.001, -65535, 65535); 

  joint.telemetrySettings.transmitPeriod = 10;
  joint.jointSettings.gearRatio = 379;

  
  CAN_InitQueues();

  // __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 10000);
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | 
                                          CAN_IT_ERROR | 
                                          CAN_IT_RX_FIFO0_OVERRUN |
                                          CAN_IT_BUSOFF|
                                          CAN_IT_ERROR_PASSIVE) != HAL_OK)
  {
	  Error_Handler();
  }





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    control_system_task();
    transmit_can_frame_task();
    transmit_telemetry_task();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
