/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void USB_CDC_RxHandler(uint8_t*, uint32_t);
void HAL_I2C_RxCallBack(void);
void Response_Handling(HAL_StatusTypeDef);
void Error_Handling(void);
//void USB_I2C_TxHandler(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t*);
//void USB_I2C_RxHandler(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t*);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SCK_Pin GPIO_PIN_10
#define I2S3_SCK_GPIO_Port GPIOC
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

// I2C Port Define
#define I2C1_PORT 0x10
#define I2C2_PORT 0x20

#define MAX_I2C_DEV 0x7f
#define MAX_I2C_REG 0x64
#define MAX_I2C_SIZE 16
#define MAX_BST_DEV_NUM 8
#define MAX_BST_REG_SIZE 2
#define DRDY_NOT_READY 0xff
#define DRDY_READY 0x00
#define END_OF_PKT1 0xee
#define END_OF_PKT2 0x0f

// USB I2C Packet Format for FUNC_READ & FUNC_WRITE modes
#define FUNC_BYTE 0
#define DEV_ADDR_BYTE 1
#define REG_BYTE 2
#define SIZE_BYTE 3
#define DATA_BYTE 4
// Min Lengths for FUNC_READ & FUNC_WRITE modes
#define PKT_MIN_RD_LEN 4
#define PKT_MIN_WR_LEN 5

// USB I2C Packet Format for FUNC_BURST modes
#define BST_FUNC_BYTE 0
#define BST_DEV_ADDR_BYTE 1
#define BST_DEV_NUM_BYTE 2
#define BST_REG_BYTE 3
#define BST_SIZE_BYTE 4
#define BST_DATA_BYTE 5
// Min Lengths for FUNC_BURST modes
#define BST_MIN_RD_LEN 5
#define BST_MIN_WR_LEN 6

// Device Ready Mode (FUNC_DEV_RDY) packet format
#define DRDY_FUNC_BYTE 0
#define DRDY_DEV_ADDR_BYTE 1
#define DRDY_DEV_NUM_BYTE 2
//Min Length for FUNC_DEV_RDY
#define DRDY_MIN_LEN 3
#define DRDY_MAX_DEV_NUM 16
#define DRDY_NUM_TRIAL 2

// Function/Modes supported for I2C1
#define FUNC_READ 0x00
#define FUNC_WRITE 0x01
#define FUNC_BURST_RD 0x02
#define FUNC_BURST_WR 0x03
#define FUNC_DEV_RDY 0x04

// Error Messages
#define GOOD 0x00 // No error at all
#define ERR0 0xe0 // Error in Device Address
#define ERR1 0xe1 // Error in I2C Register
#define ERR2 0xe2 // Error in size byte
#define ERR3 0xe3 // Error in Func mode
#define ERR4 0xe4 // Error in Length sent
#define ERR5 0xe5 // Error in HAL I2C
#define ERR6 0xe6 // Undefined function/modes
#define ERR7 0xe7 // Burst Mode Device Number over 8
#define ERR8 0xe8 // Burst Mode Reg Size over 2
#define ERR9 0xe9 // FUNC_DEV_RDY Dev Num over 16
#define ERRA 0xea // FUNC_DEV_RDY Some device is not ready
#define ERRB 0xeb // I2C Port not defined correctly

// Response Format
#define RESP_FUNC_BYTE 0
#define RESP_ERR_BYTE 1
#define RESP_DATA_BYTE 2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
