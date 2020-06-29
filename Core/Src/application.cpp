
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
I2C_HandleTypeDef * hi2c_ptr;

void setup(I2C_HandleTypeDef * hi2c_ptr_in) {
	hi2c_ptr = hi2c_ptr_in;
}

void loop(void) {
	//
	uint8_t buff[12];
	IMU.init(LSM6::deviceAuto, LSM6::sa0Auto, hi2c_ptr);
	IMU.enableDefault();
	uint8_t val = IMU.readReg(LSM6::CTRL1_XL, buff);
	uint8_t val2 = IMU.readReg(LSM6::CTRL2_G, buff);
	uint8_t val3 = IMU.readReg(LSM6::CTRL3_C, buff);
	IMU.readAcc();
}

#ifdef __cplusplus
}
#endif
