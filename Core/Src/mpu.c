/*
 * mpu6050.c
 *
 *  Created on: Dec 5, 2021
 *  Author: Kevin Fordal
 *
 *
 *
 */

#include <mpu.h>

/* Register address macros:

	WHO_AM_I
	PWR_MGMT_1
	SMPLRT_DIV
	ACCEL_XOUT_H
	ACCEL_XOUT_L
	ACCEL_YOUT_H
	ACCEL_YOUT_L
	ACCEL_ZOUT_H
	ACCEL_ZOUT_L

	MPU6050 slave address: Binary: 1101001 / 1101000. Hex: 69 / 68. Pin AD0 hi / lo.
*/

void LCD_Debug(const char* first_row, const char* second_row){ // Used for debug purposes.

	TextLCD_Puts(&lcd, first_row);
	TextLCD_Position(&lcd, 0, 1);
	TextLCD_Puts(&lcd, second_row);
	TextLCD_Home(&lcd);
}

HAL_StatusTypeDef MPU6050_Init(MPU6050_Type *sensor, I2C_HandleTypeDef *hi2c) { // Setup. OK

	HAL_StatusTypeDef ret; // Used for storing return value of the i2c functions.

	sensor->i2c = hi2c; // Set which i2c to use.
	sensor->DevAddress = MPU6050_ADDR; // Device address

	uint8_t data = 0; // Stores values to be written into the selected register.

	ret = MPU6050_WriteRegister(sensor, PWR_MGMT_1, data, 1);
	if(ret != HAL_OK) // Check return value.
		LCD_Debug("MPU INIT PWR", "MGMT FAILED"); // Display an error if not ok.

	data = 0x07; // Divide 8kHz by (1 + 0x07), 8 / 8 = 1. MPU will now update with the same frequency as its output.
	ret = MPU6050_WriteRegister(sensor, SMPLRT_DIV, data, 1);
	if(ret != HAL_OK) // Check return value.
		LCD_Debug("MPU INIT SMPLRT", "_DIV FAILED");

	data = 0x00; // 0
	ret = MPU6050_WriteRegister(sensor, ACCEL_CONFIG, data, 1);
	if(ret != HAL_OK) // Check return value.
		LCD_Debug("MPU INIT ACCEL", "CONFIG FAILED");

	if(ret == HAL_OK)
		LCD_Debug("MPU INIT:", "SUCCESSFUL");

	return ret;
}

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_Type *sensor, uint8_t reg, uint8_t *data, uint8_t bytes){ // Write to the registers.

	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Mem_Write(sensor->i2c, sensor->DevAddress, reg, 1, &data, bytes, HAL_MAX_DELAY); // Which register.
	if(ret != HAL_OK)
		LCD_Debug("MPU WRITE", "REGISTER FAILED");

	return ret;
}

HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050_Type *sensor, uint8_t address){	// Read a specific register, 2 bytes.

	HAL_StatusTypeDef ret;

	//								i2c		address		    pointer 	bytes					time
	ret = HAL_I2C_Mem_Read(sensor->i2c, sensor->DevAddress, address, 1, &sensor->data, 2, HAL_MAX_DELAY);

	if(ret != HAL_OK)
		LCD_Debug("MPU MEM READ", "REG FAILED");

	return ret;
}

HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050_Type *sensor, uint8_t address, uint8_t size){	// Read a specific register, size number of bytes.

	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Mem_Read(sensor->i2c, sensor->DevAddress, address, 1, &sensor->data, size, HAL_MAX_DELAY);

	if(ret != HAL_OK)
		LCD_Debug("MPU MEM READ", "REGS FAILED");

	return ret;
} // Read the selected register, and any following registers.
