
/*
 * application.cpp
 *
 *  Created on: Jun 29, 2020
 *      Author: Karthik
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "application.hpp"
#include "LSM6.hpp"
#include "fsm.hpp"
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

extern uint8_t spi_data[PRIMARY_SPI_BUS_DATA_SIZE_BYTES];

LSM6 IMU;

FSM OmnibotFSM;

void setup(void) {

	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_SET);

	OmnibotFSM.fsmRun();
	OmnibotFSM.fsmRun();

	IMU.init(LSM6::deviceAuto, LSM6::sa0Auto, &hi2c1);
	IMU.enableDefault();
}

void loop(void) {
	IMU.readAcc();
	IMU.readGyro();
}

void interruptLink(interruptLink_t it) {
	switch(it) {
	case SPI3_IT:
		// DO SOMETHING WITH spi_data
		// reset interrupt to top of buffer
		HAL_SPI_Receive_IT(&hspi3, spi_data, PRIMARY_SPI_BUS_DATA_SIZE_BYTES);
		break;
	case TIM4_IT:
		OmnibotFSM.fsmRun();
		break;
	default:
		break;
	}
}

void fsmRun(void) {

}

#ifdef __cplusplus
}
#endif
