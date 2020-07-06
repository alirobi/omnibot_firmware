
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

LSM6 IMU;
APP_HandleTypeDef * ptr_hOmnibotApp;

void setup(APP_HandleTypeDef * hOmnibotApp_in) {
	ptr_hOmnibotApp = hOmnibotApp_in;
	IMU.init(LSM6::deviceAuto, LSM6::sa0Auto, ptr_hOmnibotApp->hi2c1);
	IMU.enableDefault();
}

void loop(void) {
	uint8_t buff[12];
	uint8_t val = IMU.readReg(LSM6::CTRL1_XL, buff);
	uint8_t val2 = IMU.readReg(LSM6::CTRL2_G, buff);
	uint8_t val3 = IMU.readReg(LSM6::CTRL3_C, buff);
	IMU.readAcc();
	IMU.readGyro();
}

#ifdef __cplusplus
}
#endif
