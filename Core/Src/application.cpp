
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

LSM6 IMU;

void setup(void) {
	//
}

void loop(void) {
	//
	uint8_t buff[12];
	IMU.init();
	IMU.enableDefault();
	uint8_t val = IMU.readReg(LSM6::CTRL1_XL, buff);
	uint8_t val2 = IMU.readReg(LSM6::CTRL2_G, buff);
	uint8_t val3 = IMU.readReg(LSM6::CTRL3_C, buff);
	IMU.readAcc();
}

#ifdef __cplusplus
}
#endif
