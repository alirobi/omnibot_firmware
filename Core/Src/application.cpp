
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
#include "lsm6.hpp"
#include "fsm.hpp"
#include "motor.hpp"
#include "messaging.hpp"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

//extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern uint8_t sdata[SDATA_SIZE_BYTES];
extern uint8_t* const sdata_head;

void sendMessageUART(Messaging::Message* msg_buf);

void messageReaction(Messaging::Message &msg);

LSM6      IMU;
Motor     MotorA(MOTOR_A, 0, 0, 0, 0.01, 100, DIR_DEFAULT);
Motor     MotorB(MOTOR_B, 0, 0, 0, 0.01, 100, DIR_DEFAULT);
Motor     MotorC(MOTOR_C, 0, 0, 0, 0.01, 100, DIR_DEFAULT);
FSM       OmnibotFSM;
Messaging OmnibotMessaging(&sendMessageUART, &messageReaction);

/**
  * @brief  Executes general startup actions, such as initializing classes
  * @note	NOT for use in initializing internal MCU modules \ 
	*	@note See @ref FSM::coreStartup()
  * @param  none
  * @retval none
  */
void setup(void) {

	OmnibotFSM.fsmRun();
	OmnibotFSM.fsmRun();

//	IMU.init(LSM6::deviceAuto, LSM6::sa0Auto, &hi2c1);
//	IMU.enableDefault();

	MotorA.init();
	MotorB.init();
	MotorC.init();
}

/**
  * @brief  Executes actions on an un-timed loop
  * @note
  * @param  none
  * @retval none
  */
void loop(void) {
//	IMU.readAcc();
//	IMU.readGyro();
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

void dmaLink(dmaLink_t dma) {
	switch(dma) {
		case UART1_DMA:
			// HAL_UART_Transmit(&huart1, (uint8_t*)sdata, SDATA_SIZE_BYTES, 0xFFFF);
			// HAL_GPIO_TogglePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin);
			bool success = OmnibotMessaging.rxMessageSequence((uint8_t*)sdata);
			break;
//		default:
//			return;
//			break;
	}
}

void sendMessageUART(Messaging::Message* msg_buf) {
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, SDATA_SIZE_BYTES, 0xFFFF);
	return;
}

void messageReaction(Messaging::Message &msg) {
//	HAL_UART_Transmit(&huart1, (uint8_t*)sdata, SDATA_SIZE_BYTES, 0xFFFF);
	// Messaging::Message temp = msg;
	// OmnibotMessaging.sendMessage(&msg);
	// HAL_GPIO_TogglePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin);
	if (msg.msgType == Messaging::PI2NU) {
		pi2nu p2n_msg_data = *((pi2nu*)(msg.msgData));
		MotorA.setTarSpeed(p2n_msg_data.vel_a);
		MotorB.setTarSpeed(p2n_msg_data.vel_b);
		MotorC.setTarSpeed(p2n_msg_data.vel_c);
	}
	return;
}

#ifdef __cplusplus
}
#endif
