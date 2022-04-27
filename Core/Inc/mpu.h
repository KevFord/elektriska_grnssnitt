#ifndef MPU_H
#define MPU_H

#include "main.h"
#include "lcd.h"
#include <string.h>

// Register addresses:
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define	SMPLRT_DIV 0x19
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define MPU6050_ADDR 0x68 << 1

#define MPU_UPDATE_FREQ 200

TextLCDType lcd; // Easier debug bodge.

typedef struct{

	I2C_HandleTypeDef *i2c;
	uint8_t DevAddress;
	uint8_t data[6]; // 2 Reads per register, 3 registers of interest.
} MPU6050_Type;

void LCD_Debug(const char* first_row, const char* second_row); // Easier debug.

HAL_StatusTypeDef MPU6050_Init(MPU6050_Type *sensor, I2C_HandleTypeDef *hi2c); // Setup.
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_Type *sensor, uint8_t reg, uint8_t *data, uint8_t bytes); // Write to a register.
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050_Type *sensor, uint8_t address); // Read a specific register.
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050_Type *sensor, uint8_t address, uint8_t size); // Read more than one consecutive registers.

#endif /*MPU_H*/
