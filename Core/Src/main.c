/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "mpu.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMBER_OF_CALIBRATION_ITERATIONS 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

TextLCDType lcd;
MPU6050_Type mpu;

uint8_t Rec_Data[6];

int16_t Accel_X_RAW;
int16_t Accel_Y_RAW;
int16_t Accel_Z_RAW;

int16_t accelXValue;
int16_t accelYValue;
int16_t accelZValue;

HAL_StatusTypeDef ret;

uint8_t Data = 0;
uint8_t check = 0;

int16_t x_offset;
int16_t y_offset;
int16_t z_offset;

int calibrate = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void checkLevel(){

	HAL_Delay(MPU_UPDATE_FREQ); // 5 Hz update frequency.

	ret = MPU6050_ReadRegisters(&mpu, ACCEL_XOUT_H, 6);
	if(ret != HAL_OK)
		LCD_Debug("Read error:", "ACCEL_XOUT_H");

	Accel_X_RAW = (int16_t)(mpu.data[0] << 8 | mpu.data[1]); // Combine both registers into one value.
	Accel_Y_RAW = (int16_t)(mpu.data[2] << 8 | mpu.data[3]);
	Accel_Z_RAW = (int16_t)(mpu.data[4] << 8 | mpu.data[5]);

	accelXValue = Accel_X_RAW / x_offset; // Divide by the value from the calibration.
	accelYValue = Accel_Y_RAW / y_offset;
	accelZValue = Accel_Z_RAW / z_offset;

//	if(accelXValue > 0x8000){ // 2's complement.
//		accelXValue |= 0xF000;
//	}
//	if(accelYValue > 0x8000){ // 2's complement. 0x7FF
//		accelYValue |= 0xF000;
//	}
//	if(accelZValue > 0x8000){ // 2's complement.
//		accelZValue |= 0xF000;
//	}
}

void displayResultsOLED(int16_t x_val, int16_t y_val, int16_t z_val){ // Displays the calibrated values as integers.

	char strX[10];
	char strY[10];
	char strZ[10];

	sprintf(strX, "%d", x_val);// / x_offset_calibrated));
	sprintf(strY, "%d", y_val);// / y_offset_calibrated));
	sprintf(strZ, "%d", z_val);// / z_offset_calibrated));

	ssd1306_SetDisplayOn(1);
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Accel X:", Font_6x8, White);
	ssd1306_SetCursor(0, 9);
	ssd1306_WriteString(strX, Font_6x8, White);
	ssd1306_SetCursor(0, 18);
	ssd1306_WriteString("Accel Y:", Font_6x8, White);
	ssd1306_SetCursor(0, 27);
	ssd1306_WriteString(strY, Font_6x8, White);
	ssd1306_SetCursor(0, 36);
	ssd1306_WriteString("Accel Z:", Font_6x8, White);
	ssd1306_SetCursor(0, 45);
	ssd1306_WriteString(strZ, Font_6x8, White);
	ssd1306_SetCursor(0, 54);
	ssd1306_UpdateScreen();
}

void displayLevel(int16_t axis_level){ // Displays the values as a "bubble".

	int16_t tiltValue = (35 - axis_level); // Center the "bubble"

	ssd1306_SetDisplayOn(1);
	ssd1306_Fill(Black);

	for(int i = 0; i < 70; i++){ // Display some lines to indicate tilt.
		if(i % 5 == 0){
			ssd1306_SetCursor(i, 34);
			ssd1306_WriteString("|", Font_7x10, White);
		}
	}

	ssd1306_SetCursor(tiltValue, 24); // Set the "bubble".
	ssd1306_WriteString("O", Font_7x10, White);

	if(axis_level > 20 || axis_level < -20) // Just to indicate a silly angle...
		drawGarfield();
	else ssd1306_UpdateScreen();
}

float resultInDegrees(int16_t z_axis, int16_t comparative_axis){

	return (atan2(comparative_axis, z_axis) / 57.2957795);
}

void calibrateMPU(MPU6050_Type mpu6050){

	// Arrays holding the different measurements:
	int16_t x_values[NUMBER_OF_CALIBRATION_ITERATIONS];
	int16_t y_values[NUMBER_OF_CALIBRATION_ITERATIONS];
	int16_t z_values[NUMBER_OF_CALIBRATION_ITERATIONS];

	// All measurements added together:
	int16_t x_calibration;
	int16_t y_calibration;
	int16_t z_calibration;

	// Fill with samples:
	for(int i = 0; i < NUMBER_OF_CALIBRATION_ITERATIONS; i++){
		x_values[i] = Accel_X_RAW;
		y_values[i] = Accel_Y_RAW;
		z_values[i] = Accel_Z_RAW;
	}
	for(int i = 0; i < NUMBER_OF_CALIBRATION_ITERATIONS; i++){
		x_calibration += x_values[i];
		y_calibration += y_values[i];
		z_calibration += z_values[i];
	}

	// Perform the calibration, divide to find the mean value.
	x_offset = x_calibration / NUMBER_OF_CALIBRATION_ITERATIONS;
	y_offset = y_calibration / NUMBER_OF_CALIBRATION_ITERATIONS;
	z_offset = z_calibration / NUMBER_OF_CALIBRATION_ITERATIONS;

	// To preserve the sanctity of the universe, we ensure we do not divide by zero.
	if(x_offset == 0)
		x_offset = 1;
	if(y_offset == 0)
		y_offset = 1;
	if(z_offset == 0)
		z_offset = 1;

	// Let the user know that the device is being calibrated.
	ssd1306_SetDisplayOn(1);
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Calibrating...", Font_7x10, White);
	ssd1306_UpdateScreen();
	HAL_Delay(500); // A small delay so the user has time to read the text.
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == TOUCH_BTN_Pin)
		calibrate = 1;
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
  MX_USB_PCD_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  TextLCD_Init(&lcd, &hi2c1, 0x4E); // "startar" LCD
  MPU6050_Init(&mpu, &hi2c1); // Setup the mpu sensor.
  ssd1306_Init(); // OLED Init.

  float deg;
  char* str;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while(calibrate == 1){ // Check calibration flag.
		  calibrateMPU(mpu); // Calibrate if necessary.
		  calibrate = 0; // Reset the flag.
	  }
	  checkLevel(); // Take in readings from the MPU, reads x, y and z axis and combines
//	  displayResultsOLED(accelXValue, accelYValue, accelZValue); // Displays the results as integers.
	  displayLevel(accelYValue); // Displays the result in a spirit level like fashion.
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLLSAI1.PLLN = 24;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_USBCLK;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.WordLength = UART_WORDLENGTH_7B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TOUCH_BTN_Pin */
  GPIO_InitStruct.Pin = TOUCH_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TOUCH_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
