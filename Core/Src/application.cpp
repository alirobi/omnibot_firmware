
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

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

extern uint8_t sdata[SDATA_SIZE_BYTES];

LSM6 IMU;

FSM OmnibotFSM;

/**
  * @brief  Executes general startup actions, such as initializing classes
  * @note	NOT for use in initializing internal MCU modules. See @ref FSM::coreStartup()
  * @param  none
  * @retval none
  */
void setup(void) {

	OmnibotFSM.fsmRun();
	OmnibotFSM.fsmRun();

	IMU.init(LSM6::deviceAuto, LSM6::sa0Auto, &hi2c1);
	IMU.enableDefault();
}

/**
  * @brief  Executes actions on an un-timed loop
  * @note
  * @param  none
  * @retval none
  */
void loop(void) {
	IMU.readAcc();
	IMU.readGyro();
}

/**
  * @brief  Called from Interrupt Service Routines (ISRs)
  * 		running from @ref /Core/Src/stm32f4xx_it.c
  * @note	Expand as necessary
  * @param  none
  * @retval none
  */
void interruptLink(interruptLink_t it) {
	switch(it) {
	case SPI3_IT:
		// DO SOMETHING WITH spi_data
		// reset interrupt to top of buffer
//		HAL_SPI_Receive_IT(&hspi3, spi_data, SDATA_SIZE_BYTES);
		break;
	case TIM9_IT:
		// PRIMARY FSM TASK
		OmnibotFSM.fsmRun();
		break;
	default:
		break;
	}
}

#ifdef __cplusplus
}
#endif
