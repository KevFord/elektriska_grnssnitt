/*
 * mpu6050.c
 *
 *  Created on: Dec 5, 2021
 *      Author: Kevin
 */

#include "mpu6050.h"

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

void MPU6050_Init(MPU6050_Type *sensor, I2C_HandleTypeDef *hi2c) { // Setup. OK

	HAL_StatusTypeDef ret; // Used for storing return value of the i2c functions.

	sensor->hi2c = hi2c; // Set which i2c to use.
	sensor->DevAddress = 0x68 << 1; // Device address

	sensor->data[0] = PWR_MGMT_1; // Address to the power management register, need to wake it up first.

	ret = HAL_I2C_Master_Transmit(sensor->hi2c, sensor->DevAddress, &sensor->data, 1, HAL_MAX_DELAY); // Get return value.

	if(ret != HAL_OK) // Check return value.
		LCD_Debug("MPU INIT PWR", "MGMT FAILED"); // Display an error if not ok.
}

void MPU6050_WriteRegister(MPU6050_Type *sensor, uint8_t reg, uint8_t *data, int8_t bytes){ // Write to the registers.

	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Master_Transmit(sensor->hi2c, sensor->DevAddress, &reg, 1, HAL_MAX_DELAY); // Which register.
	if(ret != HAL_OK)
		LCD_Debug("MPU WRITE", "REGISTER FAILED");

	ret = HAL_I2C_Master_Transmit(sensor->hi2c, sensor->DevAddress, &data, bytes, HAL_MAX_DELAY); // Write the data.
	if(ret != HAL_OK)
		LCD_Debug("MPU WRITE", "REGISTER FAILED");
}

void MPU6050_ReadRegister(MPU6050_Type *sensor, uint8_t address){	// Read a specific register. OK

	HAL_StatusTypeDef ret;

	//								i2c					address		pointer 	bytes	time
	ret = HAL_I2C_Mem_Read(sensor->hi2c, sensor->DevAddress, address << 1, I2C_MEMADD_SIZE_16BIT, sensor->data, 2, HAL_MAX_DELAY);

	if(ret != HAL_OK)
		LCD_Debug("MPU MEM READ", "REG FAILED");
}

void MPU6050_ReadRegisters(MPU6050_Type *sensor, uint8_t address, uint8_t size){	// Read a specific register. OK

	HAL_StatusTypeDef ret;


	ret = HAL_I2C_Mem_Read(sensor->hi2c, sensor->DevAddress, address << 1, I2C_MEMADD_SIZE_16BIT, sensor->data, size, HAL_MAX_DELAY);

	if(ret != HAL_OK)
		LCD_Debug("MPU MEM READ", "REGS FAILED");
} // Read the selected register, and any following registers.

//HAL_I2C_Master_Transmit();
//HAL_I2C_Mem_Read();


