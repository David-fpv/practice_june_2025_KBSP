/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "com_packet.h"
#include "flash_lib.h"
#include "array_container.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TIM_CLK_HZ   72000000UL
#define MAX_ARR      0xFFFFU
#define MAX_PSC      0xFFFFU
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


data_package package;
array_container buffer = {
	.length = 0,
	.max_length=MAX_ARRAY_LENGHT
};


void sendPackage(uint8_t status, data_package* package, array_container* container)
{
	package->operation_code = status;
	if (package->operation_code == 0x2) package->data_length = 0; // If packet about error 
	container->length = package_to_buffer(container, package);
	container->length = add_security_symbols(container);
	HAL_UART_Transmit(&huart1, container->array, container->length, 100);
}


void sendErrorPackage(data_package* package, array_container* container)
{	
	sendPackage(0x2, package, container);
}


uint8_t analog_sensor(data_package* package, array_container* container)
{
	// something (analog sensor) - 1 byte - chennal ADC, 4 bytes - value
	// In this moment it's button - digital data
		
//package->data_length = 1;
//package->data[0] = HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_14) ? 0x1 : 0x0;
//sendPackage(0x0, package, uart_buffer, UART_MAX_BUFFER_LENGHT);
	float ADC_value_voltage;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	ADC_value_voltage = (float)HAL_ADC_GetValue(&hadc1) / 4095 * 3.3;
				
	package->data_length = 5;
	package->data[0] = 1;
	memcpy(&package->data[1], &ADC_value_voltage, 4);
	sendPackage(0x0, package, container);
	return OK;
}


uint8_t led_manage(data_package* package, array_container* container)
{
	// manage led
	if (isNormalLenghtPackage(package, 1) == ERROR)
	{
		sendErrorPackage(package, container);
		return ERROR;
	}
		
	package->data[0] > 0x0 ? HAL_GPIO_WritePin(GPIOB, LED_1_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, LED_1_Pin, GPIO_PIN_RESET);
	package->data_length = 0;	
	sendPackage(0x0, package, container);
	return OK;
}

uint8_t led_blink(data_package* package, array_container* container)
{
    if (isNormalLenghtPackage(package, 3) == ERROR) {
        sendErrorPackage(package, container);
        return ERROR;
    }

    uint8_t  duty     = package->data[0]; 
    uint16_t freq_hz  = package->data[1] << 8 | package->data[2];
    if (freq_hz == 0) freq_hz = 1;

    
    uint32_t min_psc = TIM_CLK_HZ / ((uint32_t)freq_hz * (MAX_ARR + 1));
    if (min_psc > MAX_PSC) min_psc = MAX_PSC;
    TIM2->PSC = (uint16_t)min_psc;

    uint32_t arr = TIM_CLK_HZ / ((TIM2->PSC + 1) * (uint32_t)freq_hz) - 1;
    if (arr > MAX_ARR) arr = MAX_ARR;
    TIM2->ARR = (uint16_t)arr;

    uint32_t ccr = ((uint32_t)duty * (arr + 1)) / 100;
    TIM2->CCR1 = (uint16_t)ccr;

    TIM2->EGR = TIM_EGR_UG;

		
    package->data_length = 0;
    sendPackage(0x0, package, container);
    return OK;
}

uint8_t reset(data_package* package, array_container* container)
{
	// reset
	package->data_length = 0;	
	sendPackage(0x0, package, container);
	HAL_NVIC_SystemReset();
	return OK;
}


uint8_t flash_handler(data_package* package, array_container* container)
{
	// read/write to flash
	uint32_t address;
	uint32_t data;
	
	switch (package->operation_code)
	{
		case 0x1:
			if (isNormalLenghtPackage(package, 8) == ERROR)
			{
				sendErrorPackage(package, container);
				return ERROR;
			}
			
			//address = 0x0801F800;
			memcpy(&address, &(package->data[0]), sizeof(uint32_t));
			memcpy(&data, &(package->data[4]), sizeof(uint32_t));
			address = __REV(address); //reverse address (big-endian to little-endian address)
			
			
			if (writeFlash(address, data) == HAL_OK)
			{
				package->data_length = 0;
				sendPackage(0x0, package, container);
				return OK;
			} else
			{
				sendErrorPackage(package, container);
				return ERROR;
			}
			break;
		
		case 0x2:
			if (isNormalLenghtPackage(package, 4) == ERROR)
			{
				sendErrorPackage(package, container);
				return ERROR;
			}
			
			memcpy(&address, &(package->data[0]), sizeof(uint32_t));
			address = __REV(address); //reverse address (big-endian to little-endian address)
			//address = 0x0801F800;
			data = readFlash(address);
			
			memcpy(&(package->data[4]), &data, sizeof(uint32_t));
			package->data_length = 8;
			sendPackage(0x0, package, container);
			return OK;
			break;
			
		default:
			sendErrorPackage(package, container);
			return ERROR;
	}
}


uint8_t mirror_massage_back(data_package* package, array_container* container)
{
	// Sends an unchanged message back
	sendPackage(package->operation_code, package, container);
	return OK;
}


uint8_t execute_package(data_package* package, array_container* container)
{
	switch (package->object_code)
	{
		case 0x1:
			return analog_sensor(package, container);
		
		case 0x2:
			return led_manage(package, container);
		
		case 0x3:
			return led_blink(package, container);
		
		case 0x4:
			return reset(package, container);
		
		case 0x5:
			return flash_handler(package, container);
		
		case 0x6:
			return mirror_massage_back(package, container);
		
		default:
			return ERROR;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  buffer.length = Size;
	buffer.length = remove_security_symbols(&buffer);
	buffer_to_package(&buffer, &package);
	execute_package(&package, &buffer);
	
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, buffer.array, buffer.max_length);
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, buffer.array, buffer.max_length);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, LED_2_Pin|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
