/*
 * Timer.hpp
 *
 *  Created on: Jun 24, 2020
 *      Author: Ali
 */

#ifndef INC_TIMER_HPP_
#define INC_TIMER_HPP_

#include "stm32f4xx_hal.h"

#define TIMER_CLK_FREQ 64000000U

void TIM_Init(TIM_TypeDef *TIMx);
int TIM_Set(TIM_TypeDef *TIMx, uint32_t freq, float duty);
float getDutyCycle(TIM_TypeDef *TIMx);

#endif /* INC_TIMER_HPP_ */
