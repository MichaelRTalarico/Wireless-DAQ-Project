/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @auther		   : Michael Talarico - Cheng Labs - PSU
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights resserved.</center></h2>
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
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "ad5933.h"
#include <math.h>

// Debugging options... 1 for on, 0 for off
#define DEBUG_MODE_MINIMAL			(1)
#define DEBUG_MODE_VERBOSE			(0)

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	int status = 0;

	double startFreq = 30E3;		// start frequency in Hz
	double clockFreq =  16E6;		// clock frequency in Hz
	int numSteps = 10;				// # of steps
	int iter = 0;
	double freqSteps = 2;			// frequency steps in Hz
	int settleTime = 15;
	signed short realData, imgData;
	double magnitude, gainFactor, impedance;
	double avgMag = 0;
	double avgGain = 0;
	double avgImpedance = 0;
	long calibrationImpedance = 200000;



	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_Init();

	// -------------------------------------------- AD5933 --------------------------------------------
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_LB, AD5933_RESET, 1);

	sendStr("---- CHENG LABS ----\nFlexible, wearable wireless sensor DAQ System");
	sendStr("Mar 8, 2020\t|\tBuild 1.00");
	sendStr("AD5933 Temp: "); sendNum(AD5933_GetTemperature());

	double tempStartFreq = (startFreq / (clockFreq / 4.0)) * 134217728.0;
	double tempFreqSteps = (freqSteps / (clockFreq / 4.0)) * 134217728.0;

	// Set sweep parameters
	AD5933_ConfigSweep(tempStartFreq,		// 30 kHz
			numSteps,			// 200 steps
			tempFreqSteps); 	// 2 Hz increments

	// Set settling time
	AD5933_SetRegisterValue(AD5933_NR_SETTLE_REG_HB, settleTime, 2);	// 15 cycles (x1)

#if DEBUG_MODE_VERBOSE

	// --> Checking/reporting starting frequency
	status = AD5933_GetRegisterValue(AD5933_START_FREQ_REG_HB,3);
	float retVal = ((float) status / (134217728.0)) * (4000000.0);
	sendStr("\nThe starting frequency is:");
	sendNum(retVal);

	// --> Checking/reporting number of steps
	status = AD5933_GetRegisterValue(AD5933_NR_INCR_REG_HB,2);
	sendStr("\nThe number of steps is:");
	sendNum(status);

	// --> Checking/reporting frequency increase.
	status = AD5933_GetRegisterValue(AD5933_FREQ_INCR_REG_HB,3);
	retVal = ((float) status / (134217728.0)) * (4000000.0);
	sendStr("\nThe frequency increment is:");
	sendNum(retVal);

	// --> Checking/reporting settling time.
	status = AD5933_GetRegisterValue(AD5933_NR_SETTLE_REG_HB,2);
	sendStr("\nThe number of settling cycles is:");
	sendNum(status);
#endif

	// Place AD5933 in standby
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
			AD5933_CONTROL_FUNCTION(AD5933_STANDBY),
			1);

	// OPTIONALS: Set internal clock, 2 V(P-P), and PGA gain factor to 1
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_LB, AD5933_INT_CLK, 1);
	AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, AD5933_CONTROL_RANGE(AD5933_200mVpp_RANGE), 1);
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_PGA_GAIN(AD5933_PGA_GAIN_X1), 1);


	// Initialize starting frequency
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_INIT_START_FREQ), 1);

	// 2ms delay while network analysis occurs
	HAL_Delay(2);


	// Start frequency sweep
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_START_FREQ_SWEEP), 1);

	status = 0;

	// Do this loop until the sweep is complete
	while(((status & AD5933_STATUS_SWEEP_DONE) != 4) & (iter < numSteps)) {
		// Wait for data to be valid
		do {
			status = AD5933_GetRegisterValue(AD5933_STATUS_REG,1);

			if ((status & AD5933_STATUS_DATA_VALID) != 2) {
				AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_REPEAT_FREQ), 1);
			}
		} while ((status & AD5933_STATUS_DATA_VALID) != 2);


		// Get real data
		realData = AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);

		if (realData <= 32767) {

		} else {
			realData = realData & 32767;
			realData = realData - 65536;
		}

		// Get imaginary data
		imgData  = AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);

		if (imgData <= 32767) {

		} else {
			imgData = imgData & 32767;
			imgData = imgData - 65536;
		}


		magnitude = sqrtf((realData * realData) + (imgData * imgData));


		gainFactor = 1 / (magnitude * calibrationImpedance);

		impedance = 1/(magnitude*gainFactor);

		avgMag = (avgMag + magnitude);
		avgGain = (avgGain + gainFactor);
		avgImpedance = (avgImpedance + impedance);

		AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_INCR_FREQ), 1);
		iter = iter + 1;

		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		//HAL_Delay(500);


	}

	magnitude = avgMag/numSteps;
	gainFactor = avgGain/numSteps;
	avgImpedance = avgImpedance/numSteps;


	// REPLACE CALIBRATION IMPEDANCE WITH UNKNOWN
	for (int i = 0; i < 5; i++) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(500);
	}

	while (1) {
		iter = 0;
		magnitude = 0;
		impedance = 0;
		avgImpedance = 0;
		avgMag = 0;
		while(((status & AD5933_STATUS_SWEEP_DONE) != 4) & (iter < numSteps)) {
			// Wait for data to be valid
			do {
				status = AD5933_GetRegisterValue(AD5933_STATUS_REG,1);

				if ((status & AD5933_STATUS_DATA_VALID) != 2) {
					AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_REPEAT_FREQ), 1);
				}
			} while ((status & AD5933_STATUS_DATA_VALID) != 2);


			// Get real data
			realData = AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);

			if (realData <= 32767) {

			} else {
				realData = realData & 32767;
				realData = realData - 65536;
			}

			// Get imaginary data
			imgData  = AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);

			if (imgData <= 32767) {

			} else {
				imgData = imgData & 32767;
				imgData = imgData - 65536;
			}


			magnitude = sqrtf((realData * realData) + (imgData * imgData));


			impedance = 1/(magnitude*gainFactor);

			avgMag = (avgMag + magnitude);
			avgImpedance = (avgImpedance + impedance);

			AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_INCR_FREQ), 1);
			iter = iter + 1;



		}

		magnitude = avgMag/numSteps;
		gainFactor = avgGain/numSteps;
		avgImpedance = avgImpedance/numSteps;

		HAL_Delay(10000);
	}

	sendStr("Exiting application.\n\r");
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_POWER_DOWN), 1);
	return (0);
}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
