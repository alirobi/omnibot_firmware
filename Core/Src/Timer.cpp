/*
 * Timer.cpp
 *
 *  Created on: Jun 18, 2020
 *      Author: Ali
 */

#include <Timer.hpp>
#include <stm32f4xx_hal_tim.h>
#include <stm32f4xx_hal.h>
#include "stm32f411xe.h"
//#include <stm32f3xx_hal_def.h>
//
void TIM_Init(TIM_TypeDef *TIMx){
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

//	__TIM1_CLK_ENABLE();
//	s_TimerInstance.Init.Prescaler = 32000;
//	s_TimerInstance.Init.CounterMode = TIM_COUNTERMODE_UP;
//	s_TimerInstance.Init.Period = 16254;
//	s_TimerInstance.Init.CLockDivision = TIM_CLOCKDIVISION_DIV1;
//	s_TimerInstance.Init.RepetitionCounter = 0;
//	HAL_TIM_Base_Init(&s_TimerInstance);
//	HAL_TIM_Base_Start(&s_TimerInstance);
//
//	__TIM2_CLK_ENABLE();
//	s_TimerInstance.Init.Prescaler = 32000;
//	s_TimerInstance.Init.CounterMode = TIM_COUNTERMODE_UP;
//	s_TimerInstance.Init.Period = 16254;
//	s_TimerInstance.Init.CLockDivision = TIM_CLOCKDIVISION_DIV1;
//	s_TimerInstance.Init.RepetitionCounter = 0;
//	HAL_TIM_Base_Init(&s_TimerInstance);
//	HAL_TIM_Base_Start(&s_TimerInstance);
//
//	__TIM5_CLK_ENABLE();
//	s_TimerInstance.Init.Prescaler = 32000;
//	s_TimerInstance.Init.CounterMode = TIM_COUNTERMODE_UP;
//	s_TimerInstance.Init.Period = 16254;
//	s_TimerInstance.Init.CLockDivision = TIM_CLOCKDIVISION_DIV1;
//	s_TimerInstance.Init.RepetitionCounter = 0;
//	HAL_TIM_Base_Init(&s_TimerInstance);
//	HAL_TIM_Base_Start(&s_TimerInstance);

	TIMx->CR1 &= ~TIM_CR1_CEN;
	TIMx->CR1 = TIM_CR1_CMS_1 | TIM_CR1_ARPE | TIM_CR1_CKD_0;
//	TIM_Cmd(TIM2, Enable);
}

int TIM_Set(TIM_TypeDef *TIMx, uint32_t freq, float duty){

}

float getDutyCycle(TIM_TypeDef *TIMx){

}
