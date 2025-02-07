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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t RxData[16];
uint8_t RxLen;
HAL_StatusTypeDef I2C_Status = HAL_OK;
uint8_t Status;
uint8_t Resp[MAX_I2C_SIZE + 4] = { FUNC_READ, GOOD };
uint8_t Func = FUNC_READ;
I2C_HandleTypeDef I2C_Port;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_USB_DEVICE_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_I2C_SPI_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PDM_OUT_Pin */
	GPIO_InitStruct.Pin = PDM_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : I2S3_WS_Pin */
	GPIO_InitStruct.Pin = I2S3_WS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
	GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin | SPI1_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
	 Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
	GPIO_InitStruct.Pin = I2S3_MCK_Pin | I2S3_SCK_Pin | I2S3_SD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len) {
	HAL_GPIO_TogglePin(GPIOD, LD5_Pin); // Debug

	Status = GOOD;
	RxLen = 0;

	// Define I2C Port
	if ((Buf[FUNC_BYTE] & 0xf0) == I2C1_PORT) {
		I2C_Port = hi2c1;
		Func = Buf[FUNC_BYTE] & 0x0f;
	} else if ((Buf[FUNC_BYTE] & 0xf0) == I2C2_PORT) {
		I2C_Port = hi2c2;
		Func = Buf[FUNC_BYTE] & 0x0f;
	} else {
		Status = ERRB;
		Error_Handling();
		return;
	}

	// Reseting response
	Resp[RESP_FUNC_BYTE] = Buf[FUNC_BYTE];

	// Error Checking
	if (Func == FUNC_BURST_RD || Func == FUNC_BURST_WR) {
		if ((Func == FUNC_BURST_RD && Len < BST_MIN_RD_LEN)
				|| (Func == FUNC_BURST_WR && Len < BST_MIN_WR_LEN)) {
			Status = ERR4;
		} else if (Buf[BST_DEV_ADDR_BYTE] > MAX_I2C_DEV) {
			Status = ERR0;
		} else if (Buf[BST_DEV_NUM_BYTE] > MAX_BST_DEV_NUM) {
			Status = ERR7;
		} else if (Buf[BST_REG_BYTE] > MAX_I2C_REG) {
			Status = ERR1;
		} else if (Buf[BST_SIZE_BYTE] > MAX_BST_REG_SIZE) {
			Status = ERR8;
		} else {
			if (Func == FUNC_BURST_RD) {
				for (int dev = 0; dev < Buf[BST_DEV_NUM_BYTE]; dev++) {
					I2C_Status = HAL_I2C_Mem_Read(&I2C_Port,
							(Buf[BST_DEV_ADDR_BYTE] + dev) << 1,
							Buf[BST_REG_BYTE], 1, &RxData[RxLen],
							Buf[BST_SIZE_BYTE], 1);
					RxLen = RxLen + Buf[BST_SIZE_BYTE];
					if (I2C_Status != HAL_OK) {
						Status = Status | (0b1 << dev);
					}
				}
			} else if (Func == FUNC_BURST_WR) {
				for (int dev = 0; dev < Buf[BST_DEV_NUM_BYTE]; dev++) {
					I2C_Status = HAL_I2C_Mem_Write(&I2C_Port,
							(Buf[BST_DEV_ADDR_BYTE] + dev) << 1,
							Buf[BST_REG_BYTE], 1, &Buf[BST_DATA_BYTE + RxLen],
							Buf[BST_SIZE_BYTE], 1);
					RxLen = RxLen + Buf[BST_SIZE_BYTE];
					if (I2C_Status != HAL_OK) {
						Status = Status | (0b1 << dev);
					}
				}
				memcpy(RxData, &Buf[BST_DATA_BYTE], RxLen);
			}

			if (Status == GOOD) {
				I2C_Status = HAL_OK;
			} else {
				I2C_Status = HAL_ERROR;
			}
			Response_Handling(I2C_Status);
			return;
		}
	} else if (Func == FUNC_READ || Func == FUNC_WRITE) {
		if ((Func == FUNC_READ && Len < PKT_MIN_RD_LEN)
				|| (Func == FUNC_WRITE && Len < PKT_MIN_WR_LEN)) {
			Status = ERR4;
		} else if (Buf[DEV_ADDR_BYTE] > MAX_I2C_DEV) { // I2C Device Address
			Status = ERR0;
		} else if (Buf[REG_BYTE] > MAX_I2C_REG) { // I2C Register
			Status = ERR1;
		} else if (Buf[SIZE_BYTE] > MAX_I2C_SIZE) { // Number of bytes write/read
			Status = ERR2;
		} else {
			RxLen = Buf[SIZE_BYTE];
			if (Func == FUNC_WRITE) {
				memcpy(RxData, &Buf[DATA_BYTE], RxLen);
				I2C_Status = HAL_I2C_Mem_Write(&I2C_Port,
						Buf[DEV_ADDR_BYTE] << 1, Buf[REG_BYTE], 1,
						&Buf[DATA_BYTE], Buf[SIZE_BYTE], 1);
				Response_Handling(I2C_Status);
				return;
			} else if (Func == FUNC_READ) {
				I2C_Status = HAL_I2C_Mem_Read(&I2C_Port,
						Buf[DEV_ADDR_BYTE] << 1, Buf[REG_BYTE], 1, RxData,
						Buf[SIZE_BYTE], 1);
				Response_Handling(I2C_Status);
				return;
			}
		}
	} else if (Func == FUNC_DEV_RDY) {
		if (Len < DRDY_MIN_LEN) {
			Status = ERR4;
		} else if (Buf[DRDY_DEV_ADDR_BYTE]
				+ Buf[DRDY_DEV_NUM_BYTE]> MAX_I2C_DEV) {
			Status = ERR0;
		} else if (Buf[DRDY_DEV_NUM_BYTE] > DRDY_MAX_DEV_NUM) {
			Status = ERR9;
		} else {
			for (int dev = 0; dev < Buf[DRDY_DEV_NUM_BYTE]; dev++) {
				I2C_Status = HAL_I2C_IsDeviceReady(&I2C_Port,
						(Buf[DRDY_DEV_ADDR_BYTE] + dev) << 1, DRDY_NUM_TRIAL,
						1);
				RxLen++;
				if (I2C_Status == HAL_OK) {
					RxData[dev] = DRDY_READY;
				} else {
					RxData[dev] = DRDY_NOT_READY;
					if (Status == GOOD) {
						Status = ERRA;
					}
				}
			}
			if (Status == ERRA && I2C_Status == HAL_OK) {
				I2C_Status = HAL_ERROR;
			}
			Response_Handling(I2C_Status);
			return;
		}
	} else { // Invalid Function/mode
		Status = ERR6;
	}

	// Error Response
	if (Status != GOOD) {
		HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
		Error_Handling();
	}

}

void HAL_I2C_RxCallBack(void) {
	uint8_t eop[] = { END_OF_PKT1, END_OF_PKT2 };
	HAL_GPIO_TogglePin(GPIOD, LD4_Pin); // Debug
	Resp[RESP_ERR_BYTE] = Status;
	memcpy(&Resp[RESP_DATA_BYTE], RxData, RxLen);
	memcpy(&Resp[RESP_DATA_BYTE + RxLen], eop, 2);
//	CDC_Transmit_FS(RxData, RxLen);
	CDC_Transmit_FS(Resp, RxLen + 4);
}

void Response_Handling(HAL_StatusTypeDef I2C_Status) {

	switch (I2C_Status) {
	case HAL_OK:
		HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
		if (Func == FUNC_READ || Func == FUNC_WRITE) {
			HAL_I2C_RxCallBack();
		} else if (Func == FUNC_BURST_RD || Func == FUNC_BURST_WR) {
			HAL_I2C_RxCallBack();
		} else if (Func == FUNC_DEV_RDY) {
			HAL_I2C_RxCallBack();
		}
		break;
	case HAL_BUSY:
	case HAL_ERROR:
	case HAL_TIMEOUT:
		HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
		if (Func == FUNC_READ || Func == FUNC_WRITE) {
			Status = ERR5;
			Error_Handling();
		} else if (Func == FUNC_BURST_RD || Func == FUNC_BURST_WR) {
			HAL_I2C_RxCallBack();
		} else if (Func == FUNC_DEV_RDY) {
			HAL_I2C_RxCallBack();
		}
		break;
	default:
	}
}

void Error_Handling(void) {
	uint8_t eop[] = { END_OF_PKT1, END_OF_PKT2 };
	Resp[RESP_ERR_BYTE] = Status;
	memcpy(&Resp[RESP_DATA_BYTE + RxLen], eop, 2);
	CDC_Transmit_FS(Resp, RxLen + 4);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
