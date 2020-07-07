
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

APP_HandleTypeDef * ptr_hOmnibotApp;
LSM6 IMU;

void setup(APP_HandleTypeDef * hOmnibotApp_in) {
	ptr_hOmnibotApp = hOmnibotApp_in;

	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_SET);

	HAL_TIM_Base_Start_IT(ptr_hOmnibotApp->htim4);

	IMU.init(LSM6::deviceAuto, LSM6::sa0Auto, ptr_hOmnibotApp->hi2c1);
	IMU.enableDefault();
}

void loop(void) {
	IMU.readAcc();
	IMU.readGyro();
}

#ifdef __cplusplus
}
#endif
