
/*
 * application.cpp
 *
 *  Created on: Jun 29, 2020
 *      Author: Karthik
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <application.hpp>
#include <LSM6.hpp>
#include "main.h"

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;

LSM6 IMU;

void setup(void) {

	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_SET);

	HAL_TIM_Base_Start_IT(&htim4);

	IMU.init(LSM6::deviceAuto, LSM6::sa0Auto, &hi2c1);
	IMU.enableDefault();
}

void loop(void) {
	IMU.readAcc();
	IMU.readGyro();
}

void fsmRun(void) {

}

#ifdef __cplusplus
}
#endif
