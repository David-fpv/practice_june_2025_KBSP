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
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "com_packet.h"



#define UART_MAX_BUFFER_LENGHT 50
uint8_t uart_buffer[UART_MAX_BUFFER_LENGHT] = {0};
uint16_t uart_buffer_lenght = 0;
struct data_package package;

UART_HandleTypeDef huart1;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);


void sendPackage(uint8_t status, struct data_package* package, uint8_t buffer[], uint16_t max_buffer_length)
{
	package->operation_code = status;
	uint16_t buffer_length = package_to_buffer(buffer, max_buffer_length, package);
	buffer_length = add_security_symbols(buffer, buffer_length, max_buffer_length);
	HAL_UART_Transmit(&huart1, buffer, buffer_length, 100);
}

uint8_t execute_package(struct data_package* package)
{
	switch (package->object_code)
	{
		case 0x1:
			// something (analog sensor) - 1 byte - chennal ADC, 4 bytes - value
			// In this moment it's button - digital data
			package->data_length = 1;
			package->data[0] = HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_14) ? 0x1 : 0x0;
			sendPackage(0x0, package, uart_buffer, UART_MAX_BUFFER_LENGHT);
			break;
		
		case 0x2:
			// manage led
			if (package->data_length < 1) return ERROR;
			package->data[0] > 0x0 ? HAL_GPIO_WritePin(GPIOB, LED_1_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, LED_1_Pin, GPIO_PIN_RESET);
			package->data_length = 0;	
			sendPackage(0x0, package, uart_buffer, UART_MAX_BUFFER_LENGHT);
			break;
		
		case 0x3:
			// led blink 1 Hz - 35 kHz
			break;
		
		case 0x4:
			// reset
			package->data_length = 0;	
			sendPackage(0x0, package, uart_buffer, UART_MAX_BUFFER_LENGHT);
			HAL_NVIC_SystemReset();
			break;
		
		case 0x5:
			// read/write to flash
			break;
		
		case 0x6:
			sendPackage(package->operation_code, package, uart_buffer, UART_MAX_BUFFER_LENGHT);
			break;
	}
	
	return OK;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
  MX_USART1_UART_Init();
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart_buffer, UART_MAX_BUFFER_LENGHT);
 
  while (1)
  {
		
  }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uart_buffer_lenght = Size;
		uart_buffer_lenght = remove_security_symbols(uart_buffer, uart_buffer_lenght);
		buffer_to_package(uart_buffer, uart_buffer_lenght, &package);
		execute_package(&package);
	
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart_buffer, UART_MAX_BUFFER_LENGHT);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
