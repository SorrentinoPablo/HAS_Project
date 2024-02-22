/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MOTOR_EN_GPIO_PIN  GPIO_PIN_7
#define MOTOR_EN_GPIO_PORT GPIOA
#define MOTOR_IN1_GPIO_PIN GPIO_PIN_1
#define MOTOR_IN1_GPIO_PORT GPIOB
#define MOTOR_IN2_GPIO_PIN GPIO_PIN_0
#define MOTOR_IN2_GPIO_PORT GPIOB
#define PWM_FREQUENCY 1000 // 1 kHz
#define PWM_PERIOD    100  // 100 us
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
SemaphoreHandle_t xSemaphore1;
SemaphoreHandle_t xSemaphore2;
SemaphoreHandle_t xSemaphore3;
uint8_t limitSW_State_Open;
uint8_t limitSW_State_Close;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_UART_Read_String(UART_HandleTypeDef *huart, char *str, uint8_t size);
void BT_RX(void *pvParameters);
void EN_Motor(void *pvParameters);
void Moving(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char message_rx[20];
char message_tx[20];
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Configure motor control pins as GPIO outputs
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA clock
    GPIO_InitStruct.Pin = MOTOR_EN_GPIO_PIN | MOTOR_IN1_GPIO_PIN | MOTOR_IN2_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_EN_GPIO_PORT, &GPIO_InitStruct);

    // 7 segments
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
    xSemaphore1 = xSemaphoreCreateBinary();
    xSemaphore2 = xSemaphoreCreateBinary();
    xSemaphore3 = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate((void *)BT_RX, "BT_RX", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate((void *)EN_Motor, "EN_Motor", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate((void *)Moving, "Moving", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|Alarm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 PB13
                           PB14 PB15 Alarm_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|Alarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Limit_SW1_Pin Limit_SW2_Pin Limit_SW3_Pin */
  GPIO_InitStruct.Pin = Limit_SW1_Pin|Limit_SW2_Pin|Limit_SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Switch_Pin */
  GPIO_InitStruct.Pin = Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Switch_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Tarea 1
void BT_RX(void *pvParameters) {
  while (1) {
	xSemaphoreTake(xSemaphore1, portMAX_DELAY);
	HAL_UART_Read_String(&huart3, message_rx, 20);
	sprintf(message_tx,"motor_R\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)message_tx,strlen(message_tx), HAL_MAX_DELAY);
    xSemaphoreGive(xSemaphore2);
  }
}

// Tarea 2
void EN_Motor(void *pvParameters) {
  while (1) {
    xSemaphoreTake(xSemaphore2, portMAX_DELAY);
	HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT, MOTOR_EN_GPIO_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_GPIO_PIN, GPIO_PIN_RESET);

	limitSW_State_Open = HAL_GPIO_ReadPin(Limit_SW2_GPIO_Port, Limit_SW2_Pin);
	limitSW_State_Close = HAL_GPIO_ReadPin(Limit_SW3_GPIO_Port, Limit_SW3_Pin);
    xSemaphoreGive(xSemaphore3);
  }
}

// Tarea 3
void Moving(void *pvParameters) {
  while (1) {
    xSemaphoreTake(xSemaphore3, portMAX_DELAY);
	if(limitSW_State_Open == 0){
		if(!strcmp(message_rx, "Open")){
			while(limitSW_State_Close == 1){
				HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_GPIO_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_GPIO_PIN, GPIO_PIN_RESET);
				limitSW_State_Close = HAL_GPIO_ReadPin(Limit_SW3_GPIO_Port, Limit_SW3_Pin);
			}
		}
	}
	if(limitSW_State_Close == 0){
		if(!strcmp(message_rx, "Close")){
			while(limitSW_State_Open == 1){
				HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_GPIO_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_GPIO_PIN, GPIO_PIN_SET);
				limitSW_State_Open = HAL_GPIO_ReadPin(Limit_SW2_GPIO_Port, Limit_SW2_Pin);
			}
		}
	}
	// Disable motor
	HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT, MOTOR_EN_GPIO_PIN, GPIO_PIN_RESET);
    xSemaphoreGive(xSemaphore1);
  }
}
void HAL_UART_Read_String(UART_HandleTypeDef *huart, char *str, uint8_t size){
	char caracter[1];
	uint8_t cont_rx = 0;
	do{
		HAL_UART_Receive(huart, (uint8_t*)caracter, 1, HAL_MAX_DELAY);
		str[cont_rx++] = (char)caracter[0];
		if(cont_rx >= size) break;
	}while(caracter[0] != '\n');
	str[cont_rx-2] = '\0';
	cont_rx = 0;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
