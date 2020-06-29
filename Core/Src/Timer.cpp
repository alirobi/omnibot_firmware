/*
 * Timer.cpp
 *
 *  Created on: Jun 18, 2020
 *      Author: Ali
 */

#include <Timer.hpp>
//#include <stm32f3xx_hal_tim.h>
#include <stm32f4xx_hal.h>
//#include <stm32f3xx_hal_def.h>
//
void TIM_Init(TIM_TypeDef *TIMx){
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
}

int TIM_Set(TIM_TypeDef *TIMx, uint32_t freq, float duty){

}

float getDutyCycle(TIM_TypeDef *TIMx){

}
