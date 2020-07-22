/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_W_CURRENT_MONITOR_Pin GPIO_PIN_1
#define MOTOR_W_CURRENT_MONITOR_GPIO_Port GPIOC
#define MOTOR_U_ARM_Pin GPIO_PIN_2
#define MOTOR_U_ARM_GPIO_Port GPIOC
#define MOTOR_V_ARM_Pin GPIO_PIN_3
#define MOTOR_V_ARM_GPIO_Port GPIOC
#define MOTOR_W_ENC_PHA_Pin GPIO_PIN_0
#define MOTOR_W_ENC_PHA_GPIO_Port GPIOA
#define MOTOR_W_ENC_PHB_Pin GPIO_PIN_1
#define MOTOR_W_ENC_PHB_GPIO_Port GPIOA
#define MOTOR_V_ENC_PHA_Pin GPIO_PIN_5
#define MOTOR_V_ENC_PHA_GPIO_Port GPIOA
#define MOTOR_U_CMD1_Pin GPIO_PIN_6
#define MOTOR_U_CMD1_GPIO_Port GPIOA
#define MOTOR_U_CMD2_Pin GPIO_PIN_7
#define MOTOR_U_CMD2_GPIO_Port GPIOA
#define MOTOR_V_CURRENT_MONITOR_Pin GPIO_PIN_4
#define MOTOR_V_CURRENT_MONITOR_GPIO_Port GPIOC
#define MOTOR_U_CURRENT_MONITOR_Pin GPIO_PIN_5
#define MOTOR_U_CURRENT_MONITOR_GPIO_Port GPIOC
#define MOTOR_V_CMD1_Pin GPIO_PIN_0
#define MOTOR_V_CMD1_GPIO_Port GPIOB
#define MOTOR_V_CMD2_Pin GPIO_PIN_1
#define MOTOR_V_CMD2_GPIO_Port GPIOB
#define TEST_PIN_Pin GPIO_PIN_10
#define TEST_PIN_GPIO_Port GPIOB
#define MOTOR_W_ARM_Pin GPIO_PIN_8
#define MOTOR_W_ARM_GPIO_Port GPIOC
#define MOTOR_U_ENC_PHA_Pin GPIO_PIN_8
#define MOTOR_U_ENC_PHA_GPIO_Port GPIOA
#define MOTOR_U_ENC_PHB_Pin GPIO_PIN_9
#define MOTOR_U_ENC_PHB_GPIO_Port GPIOA
#define PRIMARY_SPI_BUS_CLK_Pin GPIO_PIN_10
#define PRIMARY_SPI_BUS_CLK_GPIO_Port GPIOC
#define PRIMARY_SPI_BUS_MISO_Pin GPIO_PIN_11
#define PRIMARY_SPI_BUS_MISO_GPIO_Port GPIOC
#define PRIMARY_SPI_BUS_MOSI_Pin GPIO_PIN_12
#define PRIMARY_SPI_BUS_MOSI_GPIO_Port GPIOC
#define MOTOR_V_ENC_PHB_Pin GPIO_PIN_3
#define MOTOR_V_ENC_PHB_GPIO_Port GPIOB
#define MOTOR_W_CMD1_Pin GPIO_PIN_6
#define MOTOR_W_CMD1_GPIO_Port GPIOB
#define MOTOR_W_CMD2_Pin GPIO_PIN_7
#define MOTOR_W_CMD2_GPIO_Port GPIOB
#define I2C_BUS_SCL_Pin GPIO_PIN_8
#define I2C_BUS_SCL_GPIO_Port GPIOB
#define I2C_BUS_SDA_Pin GPIO_PIN_9
#define I2C_BUS_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SDATA_SIZE_BYTES 64

#define MOTOR_U_CMD1_TIMER &htim3
#define MOTOR_U_CMD1_CHANNEL TIM_CHANNEL_1
#define MOTOR_U_CMD2_TIMER &htim3
#define MOTOR_U_CMD2_CHANNEL TIM_CHANNEL_2

#define MOTOR_U_ENC_TIM &htim1

#define MOTOR_V_CMD1_TIMER &htim3
#define MOTOR_V_CMD1_CHANNEL TIM_CHANNEL_3
#define MOTOR_V_CMD2_TIMER &htim3
#define MOTOR_V_CMD2_CHANNEL TIM_CHANNEL_4

#define MOTOR_V_ENC_TIM &htim2

#define MOTOR_W_CMD1_TIMER &htim4
#define MOTOR_W_CMD1_CHANNEL TIM_CHANNEL_1
#define MOTOR_W_CMD2_TIMER &htim4
#define MOTOR_W_CMD2_CHANNEL TIM_CHANNEL_2

#define MOTOR_W_ENC_TIM &htim5

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
