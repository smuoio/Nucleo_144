/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "Util.h"
#include "Config.h"
#include "LedManager.h"

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
ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

SPI_HandleTypeDef hspi2;

PCD_HandleTypeDef hpcd_USB_OTG_FS;


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

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTrask */
osThreadId_t ControlTraskHandle;
uint32_t ControlTraskBuffer[ 128 ];
osStaticThreadDef_t ControlTraskControlBlock;
const osThreadAttr_t ControlTrask_attributes = {
  .name = "ControlTrask",
  .stack_mem = &ControlTraskBuffer[0],
  .stack_size = sizeof(ControlTraskBuffer),
  .cb_mem = &ControlTraskControlBlock,
  .cb_size = sizeof(ControlTraskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DiagnosticTask */
osThreadId_t DiagnosticTaskHandle;
uint32_t DiagnosticTaskBuffer[ 128 ];
osStaticThreadDef_t DiagnosticTaskControlBlock;
const osThreadAttr_t DiagnosticTask_attributes = {
  .name = "DiagnosticTask",
  .stack_mem = &DiagnosticTaskBuffer[0],
  .stack_size = sizeof(DiagnosticTaskBuffer),
  .cb_mem = &DiagnosticTaskControlBlock,
  .cb_size = sizeof(DiagnosticTaskControlBlock),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ConsoleTask */
osThreadId_t ConsoleTaskHandle;
uint32_t ConsoleTaskBuffer[ 128 ];
osStaticThreadDef_t ConsoleTaskControlBlock;
const osThreadAttr_t ConsoleTask_attributes = {
  .name = "ConsoleTask",
  .stack_mem = &ConsoleTaskBuffer[0],
  .stack_size = sizeof(ConsoleTaskBuffer),
  .cb_mem = &ConsoleTaskControlBlock,
  .cb_size = sizeof(ConsoleTaskControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EthMutex */
osMutexId_t EthMutexHandle;
osStaticMutexDef_t EthMutexControlBlock;
const osMutexAttr_t EthMutex_attributes = {
  .name = "EthMutex",
  .cb_mem = &EthMutexControlBlock,
  .cb_size = sizeof(EthMutexControlBlock),
};
/* Definitions for ControlMutex */
osMutexId_t ControlMutexHandle;
osStaticMutexDef_t ControlMutexControlBlock;
const osMutexAttr_t ControlMutex_attributes = {
  .name = "ControlMutex",
  .cb_mem = &ControlMutexControlBlock,
  .cb_size = sizeof(ControlMutexControlBlock),
};
/* Definitions for DiagnosticMutex */
osMutexId_t DiagnosticMutexHandle;
osStaticMutexDef_t DiagnosticMutexControlBlock;
const osMutexAttr_t DiagnosticMutex_attributes = {
  .name = "DiagnosticMutex",
  .cb_mem = &DiagnosticMutexControlBlock,
  .cb_size = sizeof(DiagnosticMutexControlBlock),
};
/* Definitions for ConsoleMutex */
osMutexId_t ConsoleMutexHandle;
osStaticMutexDef_t ConsoleMutexControlBlock;
const osMutexAttr_t ConsoleMutex_attributes = {
  .name = "ConsoleMutex",
  .cb_mem = &ConsoleMutexControlBlock,
  .cb_size = sizeof(ConsoleMutexControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ETH_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);
void MainControl(void const * argument);
void MainDiagnostic(void const * argument);
void MainConsoleTask(void const * argument);
static void testled(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI2_Init();
  MX_FATFS_Init();
 // MX_USB_OTG_FS_PCD_Init();
 // MX_ETH_Init();
  ConfigInit(); // configuration init

  /* USER CODE BEGIN 2 */
  /* Create the mutex(es) */
  /* definition and creation of EthMutex */
  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of EthMutex */
  EthMutexHandle = osMutexNew(&EthMutex_attributes);

  /* creation of ControlMutex */
  ControlMutexHandle = osMutexNew(&ControlMutex_attributes);

  /* creation of DiagnosticMutex */
  DiagnosticMutexHandle = osMutexNew(&DiagnosticMutex_attributes);

  /* creation of ConsoleMutex */
  ConsoleMutexHandle = osMutexNew(&ConsoleMutex_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ControlTrask */
  //ControlTraskHandle = osThreadNew(MainControl, NULL, &ControlTrask_attributes);

  /* creation of DiagnosticTask */
  //DiagnosticTaskHandle = osThreadNew(MainDiagnostic, NULL, &DiagnosticTask_attributes);

  /* creation of ConsoleTask */
//  ConsoleTaskHandle = osThreadNew(MainConsoleTask, NULL, &ConsoleTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  //GPIOB->ODR|=GPIO_ODR_OD7;
	 // if(HAL_UART_Transmit(&huart3, (uint8_t*)"STM32F767ZI Loop\n\r", 15, 1000) == HAL_OK)
	 //    leddebug();
	  //printf("[main Loop]::while(1)\n\r");
	  //HAL_Delay(500);
	  //ledmanager();
	  //GPIOB->ODR&=~GPIO_ODR_OD7;
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
	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	  /** Configure LSE Drive Capability
	  */
	  HAL_PWR_EnableBkUpAccess();
	  /** Configure the main internal regulator output voltage
	  */
	  __HAL_RCC_PWR_CLK_ENABLE();
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 4;
	  RCC_OscInitStruct.PLL.PLLN = 96;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = 4;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Activate the Over-Drive mode
	  */
	  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
	  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */

static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.Speed = ETH_SPEED_100M;
  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOG_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED3_Pin|LED2_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(PinDebug_GPIO_Port, PinDebug_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : Blue_Button_Pin */
	  GPIO_InitStruct.Pin = Blue_Button_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Blue_Button_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : LED1_Pin LED3_Pin LED2_Pin */
	  GPIO_InitStruct.Pin = LED1_Pin|LED3_Pin|LED2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : USB_OverCurrent_Pin */
	  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PinDebug_Pin */
	  GPIO_InitStruct.Pin = PinDebug_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(PinDebug_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

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
  char *msg = "StartDefaultTask!\n\r";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 0xFFFF);
  for(;;)
  {
	  ControlTraskHandle = osThreadNew(MainControl, NULL, &ControlTrask_attributes);
	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 0xFFFF);
	  osDelay(1);
	  osThreadExit();
	  //Terminate the task
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MainControl */
/**
* @brief Function implementing the ControlTrask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MainControl */
void MainControl(void const * argument)
{
  /* USER CODE BEGIN MainControl */
  /* Infinite loop */
  char *msg = "MainControlTask!\n\r";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 0xFFFF);
  DiagnosticTaskHandle = osThreadNew(MainDiagnostic, NULL, &DiagnosticTask_attributes);
  for(;;)
  {
	  // entry mutex
	osMutexWait(ControlMutexHandle, 0);
	//set a pin to verify the right works
	HAL_GPIO_TogglePin(PinDebug_GPIO_Port, PinDebug_Pin);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	//HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 0xFFFF);
    osDelay(50);
    osMutexRelease(ControlMutexHandle);
     // exit mutex
  }
  /* USER CODE END MainControl */
}

/* USER CODE BEGIN Header_MainDiagnostic */
/**
* @brief Function implementing the DiagnosticTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MainDiagnostic */
void MainDiagnostic(void const * argument)
{
  /* USER CODE BEGIN MainDiagnostic */
  char *msg = "MainDiagnosticTask!\n\r";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 0xFFFF);
  ConsoleTaskHandle = osThreadNew(MainConsoleTask, NULL, &ConsoleTask_attributes);
  /* Infinite loop */
  for(;;)
  {
	osMutexWait(DiagnosticMutexHandle, 0);
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    osDelay(200);
    osMutexRelease(DiagnosticMutexHandle);
  }
  /* USER CODE END MainDiagnostic */
}

/* USER CODE BEGIN Header_MainConsoleTask */
/**
* @brief Function implementing the ConsoleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MainConsoleTask */
void MainConsoleTask(void const * argument)
{
  /* USER CODE BEGIN MainConsoleTask */
  char *msg = "MainConsoleTask!\n\r";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 0xFFFF);
  /* Infinite loop */
  for(;;)
  {
	  osMutexWait(ConsoleMutexHandle, 0);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  osDelay(500);
	  osMutexRelease(ConsoleMutexHandle);
  }
  /* USER CODE END MainConsoleTask */
}


void testled(void)
{
	while(1)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
