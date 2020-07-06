/*
 * application.hpp
 *
 *  Created on: Jun 29, 2020
 *      Author: Karthik
 */

#ifndef INC_APPLICATION_HPP_
#define INC_APPLICATION_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef struct {
	ADC_HandleTypeDef 	*hadc1;

	I2C_HandleTypeDef 	*hi2c1;

	SPI_HandleTypeDef 	*hspi3;

	TIM_HandleTypeDef 	*htim1;
	TIM_HandleTypeDef 	*htim2;
	TIM_HandleTypeDef 	*htim3;
	TIM_HandleTypeDef 	*htim5;
	TIM_HandleTypeDef 	*htim9;

} APP_HandleTypeDef;

void setup(APP_HandleTypeDef * hOmnibotApp_in);

void loop(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_APPLICATION_HPP_ */
