/**
 ******************************************************************************
 * File Name          : USART.c
 * Description        : This file provides code for the configuration
 *                      of the USART instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}

}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (uartHandle->Instance == USART1) {
		/* USER CODE BEGIN USART1_MspInit 0 */

		/* USER CODE END USART1_MspInit 0 */
		/* USART1 clock enable */
		__HAL_RCC_USART1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10     ------> USART1_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN USART1_MspInit 1 */

		/* USER CODE END USART1_MspInit 1 */
	}
}
//--

/**
 * @brief  Function to send strings over USART
 * @retval void
 */
void sendStr(char _str[]) {

	HAL_UART_Transmit(&huart1, (uint8_t*) _str, strlen(_str), HAL_MAX_DELAY);
	HAL_Delay(10);
}

/**
 * @brief  Function to send integers and floating point numbers over USART
 * @retval void
 */
void sendNum(float num) {
	char buf[128] = { 0 };
	char *buff = (char*) &buf;
	uint16_t size = 0;

	// Code structure recommended by Escher (https://stackoverflow.com/questions/23191203/convert-float-to-string-without-sprintf)
	// Modified for my own purpose
	char *s = buff + 128;
	uint32_t decimals;
	int units;
	if (num < 0) {         // - numbers
		decimals = (int) (num * -1000) % 1000; // 3 decimals chosen
		units = (int) (-1 * num);
	} else {            // + numbers
		decimals = (int) (num * 1000) % 1000;
		units = (int) num;
	}
	if (units < 10000) { // Ignored decimal for numbers that are large, as algorithm becomes inefficient
		for (int i = 0; i < 3; i++) {
			*--s = (decimals % 10) + '0';
			decimals /= 10;
			size++;
		}
		*--s = '.';
		size++;
	}
	while (units > 0) {
		*--s = (units % 10) + '0';
		units /= 10;
		size++;
	}
	if (num < 0) {
		*--s = '-';
		size++; // unary minus sign for negative numbers
	}
	if (HAL_UART_Transmit(&huart1, (uint8_t*) s, size, HAL_MAX_DELAY)
			!= HAL_OK) {
		Error_Handler();
	}
	HAL_Delay(10);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle) {

	if (uartHandle->Instance == USART1) {
		/* USER CODE BEGIN USART1_MspDeInit 0 */

		/* USER CODE END USART1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10     ------> USART1_RX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

		/* USER CODE BEGIN USART1_MspDeInit 1 */

		/* USER CODE END USART1_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
