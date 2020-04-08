/***************************************************************************//**
 *   @file   AD5933.c
 *   @brief  AD5933 Driver.
 *   @author ATofan (alexandru.tofan@analog.com)
 ********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************
 *   SVN Revision: $WCREV$
 *******************************************************************************/

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <ad5933.h>
#include <stdio.h>
#include <math.h>
#include "I2C.h"
#include "usart.h"


// Debugging options... 1 for on, 0 for off
#define DEBUG_MODE_VERBOSE			(1)

int temperature = 0;
unsigned long impedanceKohms = 0;
unsigned long impedanceOhms = 0;
float impedance = 0.0f;
float gainFactor = 0.0f;

// Default values for gain and range
unsigned int chosenGain = AD5933_PGA_GAIN_X1;
unsigned int chosenRange = AD5933_2Vpp_RANGE;

double clockFreq = 16E6;		// clock frequency in Hz

/******************************************************************************
 * @brief Set an AD5933 internal register value.
 *
 * @param registerAddress - Address of AD5933 register.
 *
 * @param registerValue - Value of data to be written in the register.
 *
 * @param numberOfBytes - Number of bytes to be written in the register
 *
 * @return None.
 ******************************************************************************/
void AD5933_SetRegisterValue(int registerAddress, int registerValue,
		char numberOfBytes) {
	char byte = 0;
	unsigned char writeData[2] = { 0, 0 };

	// Write numberOfBytes from starting address to starting address + numberOfBytes
	for (byte = 0; byte < numberOfBytes; byte++) {
		writeData[0] = registerAddress + numberOfBytes - byte - 1;
		writeData[1] = ((registerValue >> (8 * byte)) & 0xFF);

		if (HAL_I2C_Master_Transmit(&hi2c1, (AD5933_I2C_ADDR << 1), writeData,
				2, HAL_MAX_DELAY) != HAL_OK) {
			Error_Handler();
		}
	}

	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		HAL_Delay(1);
	}
}

/******************************************************************************
 * @brief Read an AD5933 internal register value.
 *
 * @param registerAddress - Address of AD5933 register.
 *
 * @param numberOfBytes - Number of bytes to be read from the register.
 *
 * @return Register value.
 ******************************************************************************/
int AD5933_GetRegisterValue(int registerAddress, char numberOfBytes) {
	int registerValue = 0;
	char byte = 0;
	unsigned char writeData[2] = { 0, 0 };
	unsigned char readData[2] = { 0, 0 };

	for (byte = 0; byte < numberOfBytes; byte++) {
		// Set pointer to desired register
		writeData[0] = AD5933_ADDR_POINTER;
		writeData[1] = registerAddress + byte;

		if (HAL_I2C_Master_Transmit(&hi2c1, (AD5933_I2C_ADDR << 1), writeData,
				2, HAL_MAX_DELAY) != HAL_OK) {
			Error_Handler();
		}

		// Read register data
		if (HAL_I2C_Master_Receive(&hi2c1, (AD5933_I2C_ADDR << 1), readData, 1,
				HAL_MAX_DELAY) != HAL_OK) {
			Error_Handler();
		}
		registerValue |= readData[0] << (8 * (numberOfBytes - byte - 1));
	}

	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	}

	return (registerValue);
}

/******************************************************************************
 * @brief Read the AD5933 temperature.
 *
 * @param None.
 *
 * @return Temperature value.
 ******************************************************************************/
int AD5933_GetTemperature(void) {
	int temperature = 0;
	int status = 0;

	// Enable temperature measurement
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
			AD5933_CONTROL_FUNCTION(AD5933_MEAS_TEMP), 1);
	// Wait for read temperature to be valid
	while ((status & AD5933_STATUS_TEMP_VALID) == 0) {
		status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
	}

	// Read correct temperature value
	temperature = AD5933_GetRegisterValue(AD5933_TEMP_REG_HB, 2);

	// Calculate temperature according to datasheet specifications
	if (temperature < 8192) {
		temperature /= 32;
	} else {
		temperature -= 16384;
		temperature /= 32;
	}

	return (temperature);
}

/******************************************************************************
 * @brief Set the gain and voltage Ouput range
 *
 * @param gain - Number of increment steps.
 *
 * @param range - Starting frequency value.
 *
 * @return None.
 ******************************************************************************/
void AD5933_SetPGAandRange(int gain, int range) {
	chosenGain = gain;
	chosenRange = range;

	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_PGA_GAIN(gain), 1);

	AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB,
			AD5933_CONTROL_RANGE(range), 1);
}

/******************************************************************************
 * @brief Configure the AD5933 frequency sweep parameters.
 *
 * @param startFreq - Starting frequency value.
 *
 * @param incSteps - Number of increment steps.
 *
 * @param incFreq - Frequency step value.
 *
 * @return None.
 ******************************************************************************/
void AD5933_ConfigSweep(int startFreq, int incSteps, int incFreq, int setTime) {
	// Ensure that incNum is a valid data.
	if (incSteps > 511) {
		incSteps = 511;
	}

	double tempStartFreq = (startFreq / (clockFreq / 4.0)) * TWO_POW_27
			;
	double tempIncFreq = (incFreq / (clockFreq / 4.0)) * TWO_POW_27
			;

	// Configure starting frequency
	AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, tempStartFreq, 3);

	// Configure number of steps
	AD5933_SetRegisterValue(AD5933_NR_INCR_REG_HB, incSteps, 2);

	// Configure frequency increment step
	AD5933_SetRegisterValue(AD5933_FREQ_INCR_REG_HB, tempIncFreq, 3);

	// Configure settle time
	AD5933_SetRegisterValue(AD5933_NR_SETTLE_REG_HB, setTime, 2);// 15 cycles (x1)

#if DEBUG_MODE_VERBOSE
	int status = 0;

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
}

/******************************************************************************
 * @brief Start AD5933 frequency sweep.
 *
 * @param None.
 *
 * @return None.
 ******************************************************************************/
void AD5933_StartSweep(void) {
	int status = 0;

	// Place AD5933 in standby
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
			AD5933_CONTROL_FUNCTION(AD5933_STANDBY), 1);

	// Select internal system clock
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_LB,
			AD5933_INT_CLK, 1);

	// Reset AD5933
	AD5933_Reset();

	// Initialize starting frequency
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
			AD5933_CONTROL_FUNCTION(AD5933_INIT_START_FREQ), 1);

	// Start frequency sweep
	AD5933_SetRegisterValue(
			AD5933_CONTROL_REG_HB,
			AD5933_CONTROL_FUNCTION(AD5933_START_FREQ_SWEEP), 1);

	// Wait for data to be valid
	while ((status & AD5933_STATUS_DATA_VALID) == 0) {
		status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
	}
}

/******************************************************************************
 * @brief Calculate gain factor
 *
 * @param calibrationImpedance - Known value of connected impedance for calibration.
 *
 * @param freqFunction - Select Repeat Frequency Sweep.
 *
 * @return gainFactor.
 ******************************************************************************/
double AD5933_CalculateGainFactor(long calibrationImpedance,
		int freqFunction) {
	double gainFactor = 0;
	double magnitude = 0;
	int status = 0;
	signed short realData = 0;
	signed short imgData = 0;

	// Repeat frequency sweep with last set parameters
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
			AD5933_CONTROL_FUNCTION(freqFunction), 1);

	// Wait for data received to be valid
	while ((status & AD5933_STATUS_DATA_VALID) == 0) {
		status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
	}

	// Read real and imaginary data
	realData = AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);

	imgData = AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);

	// Calculate magnitude
	magnitude = sqrtf((realData * realData) + (imgData * imgData));

	// Calculate gain factor
	gainFactor = 1 / (magnitude * calibrationImpedance);

	return (gainFactor);
}

/******************************************************************************
 * @brief Calculate impedance.
 *
 * @param gainFactor - Gain factor calculated using a known impedance.
 *
 * @param freqFunction - Select Repeat Frequency Sweep.
 *
 * @return impedance.
 ******************************************************************************/
double AD5933_CalculateImpedance(double gainFactor, char freqFunction) {
	signed short realData = 0;
	signed short imgData = 0;
	double magnitude = 0;
	double impedance = 0;
	int status = 0;

	// Repeat frequency sweep with last set parameters
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
			AD5933_CONTROL_FUNCTION(freqFunction), 1);

	// Wait for data received to be valid
	while ((status & AD5933_STATUS_DATA_VALID) == 0) {
		status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
	}

	// Read real and imaginary data
	realData = AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);
	//sendStr("Impedance measurement real: ");
	//sendNum(realData);

	imgData = AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);
	//sendStr("Impedance measurement img: ");
	//sendNum(imgData);

	// Calculate magnitude
	magnitude = sqrtf((realData * realData) + (imgData * imgData));

	// Calculate impedance
	impedance = 1 /(magnitude * gainFactor);

	return (impedance);
}

/***************************************************************************
 * @brief Places the device in standby mode.
 *
 * @return None.
 ***************************************************************************/
void AD5933_StandBy(void) {
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
			AD5933_CONTROL_FUNCTION(AD5933_STANDBY), 1);
}

/***************************************************************************
 * @brief Resets the device.
 *
 * @return None.
 ***************************************************************************/
void AD5933_Reset(void) {

	AD5933_SetRegisterValue(AD5933_CONTROL_REG_LB,
			AD5933_RESET, 1);
}

/***************************************************************************
 * @brief Power the device off.
 *
 * @return None.
 ***************************************************************************/
void AD5933_Off() {
	AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
			AD5933_CONTROL_FUNCTION(AD5933_POWER_DOWN), 1);
}

