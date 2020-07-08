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

void setup(void);

void loop(void);

void fsmRun(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_APPLICATION_HPP_ */
