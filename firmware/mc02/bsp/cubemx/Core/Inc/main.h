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
#include "stm32h7xx_hal.h"

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
// libhcs: app.cpp provides its own main() and drives the generated init/clock
// functions directly, so expose them here. Kept inside USER CODE so CubeMX
// regeneration preserves the declarations.
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
// libhcs: MPU_Config() cannot be declared here. CubeMX emits itsprototype in
// main.c as `static void MPU_Config(void);`, outside any USERCODE block, so it
// comes back on every Generate; a non-static declaration reachingmain.c ahead
// of it makes GCC reject the file outright. The wrapper below isefined in
// main.c's USER CODE BEGIN 4 (which Generate preserves) and callsit from
// inside that translation unit, where the static prototype is in scope.
void libhcs_mpu_config(void);

// libhcs: called from the fault handlers in stm32h7xx_it.c. Requests DFU
// through the boot mailbox and resets, so an application that faults cannot
// leave the board bricked in an unreachable while(1). Defined per target
// (app and bootloader each provide their own), hence the C linkage here.
void libhcs_fault_recover(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Power_OUT2_EN_Pin GPIO_PIN_13
#define Power_OUT2_EN_GPIO_Port GPIOC
#define Power_OUT1_EN_Pin GPIO_PIN_14
#define Power_OUT1_EN_GPIO_Port GPIOC
#define Power_5V_EN_Pin GPIO_PIN_15
#define Power_5V_EN_GPIO_Port GPIOC
#define CS1_ACCEL_Pin GPIO_PIN_0
#define CS1_ACCEL_GPIO_Port GPIOC
#define BMI088_MOSI_Pin GPIO_PIN_1
#define BMI088_MOSI_GPIO_Port GPIOC
#define BMI088_MISO_Pin GPIO_PIN_2
#define BMI088_MISO_GPIO_Port GPIOC
#define CS1_GYRO_Pin GPIO_PIN_3
#define CS1_GYRO_GPIO_Port GPIOC
#define DCMI_PWDN_Pin GPIO_PIN_5
#define DCMI_PWDN_GPIO_Port GPIOC
#define INT1_ACC_Pin GPIO_PIN_10
#define INT1_ACC_GPIO_Port GPIOE
#define INT1_ACC_EXTI_IRQn EXTI15_10_IRQn
#define INT1_GYRO_Pin GPIO_PIN_12
#define INT1_GYRO_GPIO_Port GPIOE
#define INT1_GYRO_EXTI_IRQn EXTI15_10_IRQn
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOE
#define LCD_BLK_Pin GPIO_PIN_10
#define LCD_BLK_GPIO_Port GPIOB
#define LCD_RES_Pin GPIO_PIN_11
#define LCD_RES_GPIO_Port GPIOB
#define DCMI_REST_Pin GPIO_PIN_12
#define DCMI_REST_GPIO_Port GPIOB
#define BMI088_SCK_Pin GPIO_PIN_13
#define BMI088_SCK_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_10
#define LCD_DC_GPIO_Port GPIOD
#define KEY_Pin GPIO_PIN_15
#define KEY_GPIO_Port GPIOA
#define EXT_SPI3_CLK_Pin GPIO_PIN_10
#define EXT_SPI3_CLK_GPIO_Port GPIOC
#define EXT_SPI3_MISO_Pin GPIO_PIN_11
#define EXT_SPI3_MISO_GPIO_Port GPIOC
#define DCMI_SCL_Pin GPIO_PIN_8
#define DCMI_SCL_GPIO_Port GPIOB
#define DCMI_SDA_Pin GPIO_PIN_9
#define DCMI_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
