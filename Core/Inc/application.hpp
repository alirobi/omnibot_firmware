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

void setup(I2C_HandleTypeDef * hi2c_ptr_in);

void loop(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_APPLICATION_HPP_ */
