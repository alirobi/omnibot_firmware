/*
 * fsm.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: Karthik
 */

#include "fsm.hpp"
#include "motor.hpp"
#include "messaging.hpp"
#include "main.h"
#include <string.h>

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

extern Motor MotorA;
extern Motor MotorB;
extern Motor MotorC;

extern Messaging OmnibotMessaging;

/**
	* @brief  FSM constructor. Sets state to disabled
	* @note
	* @param  none
	* @retval none
	*/
FSM::FSM(void) : curState_{DISABLED} {}


/**
	* @brief  Runs the FSM for one step
	* @note
	* @param  none
	* @retval successful execution of FSM step
	*/
bool FSM::fsmRun(void) {
	if (busy_) return false;
	busy_ = true;
	switch(curState_) {
		case DISABLED:
			fsmDisabled();
			break;
		case CORE_STARTUP:
			fsmCoreStartup();
			break;
		case CONFIG:
			fsmConfig();
			break;
		case AUX_STARTUP:
			fsmAuxStartup();
			break;
		case CALIBRATE:
			fsmCalibrate();
			break;
		case DRIVE:
			fsmDrive();
			break;
		case STOP_IDLE:
			fsmStopIdle();
			break;
		case FAULT:
			fsmFault();
			break;
		default:
			fsmTransition(STOP_IDLE);
			break;
	}
	busy_ = false;
	return true;
}

/**
	* @brief  Transitions FSM state
	* @note   Not all transitions are valid
	* @note	DOES NOT trigger execution of FSM step
	* @param  desired next state from @ref FSM::fsmState_t
	* @retval valid state transition
	*/
bool FSM::fsmTransition(fsmState_t nextState) {
	bool validity = INVALID_TRANS;
	if (nextState == curState_) return VALID_TRANS;
	switch(nextState) {
		case DISABLED:
			if (curState_ == STOP_IDLE) {
				// FSM will start in DISABLE, but otherwise should only get here
				// after a STOP_IDLE routine
				validity = VALID_TRANS;
			}
			break;
		case CORE_STARTUP:
			if (curState_ == DISABLED) {
				validity = VALID_TRANS;
			}
			break;
		case CONFIG:
			if (curState_ == STOP_IDLE) {
				// CONFIG should only occur after a STOP
				validity = VALID_TRANS;
			}
			break;
		case AUX_STARTUP:
			if (curState_ == STOP_IDLE) {
				validity = VALID_TRANS;
			}
			break;
		case CALIBRATE:
			if (curState_ == AUX_STARTUP || curState_ == STOP_IDLE) {
				validity = VALID_TRANS;
			}
			break;
		case DRIVE:
			if (curState_ == AUX_STARTUP || curState_ == CALIBRATE) {
				validity = VALID_TRANS;
			}
			break;
		case STOP_IDLE:
			validity = VALID_TRANS;
			break;
		case FAULT:
			validity = VALID_TRANS;
			break;
		default:
			break;
	}
	if (validity) {
		curState_ = nextState;
	}
	return validity;
}

FSM::fsmState_t FSM::getCurState() {
	return curState_;
}


//TODO: Implement these properly:

/**
	* @brief  Executes actions to be taken at every execution in the DISABLED state
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmDisabled(void) {
	fsmTransition(CORE_STARTUP);
}

/**
	* @brief  Executes core startup actions, such as MCU module initializations and startups
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmCoreStartup(void) {

	// Motor encoders ready to read
	HAL_TIM_Encoder_Start(MOTOR_U_ENC_TIM, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(MOTOR_V_ENC_TIM, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(MOTOR_W_ENC_TIM, TIM_CHANNEL_ALL);

	// Motors disarmed
	MotorA.disarm();
	MotorB.disarm();
	MotorC.disarm();

	// Motor command PWM generation -- initialized at 0% duty cycle
	HAL_TIM_PWM_Start(MOTOR_U_CMD1_TIMER, MOTOR_U_CMD1_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_U_CMD2_TIMER, MOTOR_U_CMD2_CHANNEL);

	HAL_TIM_PWM_Start(MOTOR_V_CMD1_TIMER, MOTOR_V_CMD1_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_V_CMD2_TIMER, MOTOR_V_CMD2_CHANNEL);

	HAL_TIM_PWM_Start(MOTOR_W_CMD1_TIMER, MOTOR_W_CMD1_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_W_CMD2_TIMER, MOTOR_W_CMD2_CHANNEL);

	// Start timer for timed interrupt for FSM task
	HAL_TIM_Base_Start_IT(&htim9);

	// Auto transition
	fsmTransition(STOP_IDLE);
}

/**
	* @brief  Configures MCU and peripherals according to new config
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmConfig(void) {
	HAL_TIM_Base_Stop_IT(&htim9);
	fsmTransition(STOP_IDLE);
	HAL_TIM_Base_Start_IT(&htim9);
}

/**
	* @brief  Executes actions to startup auxiliary devices (e.g., motor drivers)
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmAuxStartup(void) {
	MotorA.arm();
	MotorB.arm();
	MotorC.arm();

	float p = 0.0000; // amount of command to increase by for every count per step in error
	float i = 0.00000;
	float d = 0.00000;

	MotorA.setPID(p, i, d);
	MotorB.setPID(p, i, d);
	MotorC.setPID(p, i, d);
	fsmTransition(CALIBRATE);
}

void FSM::fsmCalibrate(void) {
	// bool calA = MotorA.calibrate();
	// bool calB = MotorB.calibrate();
	// bool calC = MotorC.calibrate();

	// if (calA && calB && calC) {
	// 	MotorA.manualCommand(0);
	// 	MotorB.manualCommand(0);
	// 	MotorC.manualCommand(0);
	// 	fsmTransition(DRIVE);
	// }
	static bool calA;
	static bool calB;
	static bool calC;
	static int8_t i = 0;
	while (i < 127) {
		calA = MotorA.calibrateToSpeed(i);
		calB = MotorB.calibrateToSpeed(i);
		calC = MotorC.calibrateToSpeed(i);
		if (!calA || !calB || !calC) return;
		MotorA.calibrateReset();
		MotorB.calibrateReset();
		MotorC.calibrateReset();
		++i;
		return;
	}
	MotorA.manualCommand(0);
	MotorB.manualCommand(0);
	MotorC.manualCommand(0);
	i = 0;
	fsmTransition(DRIVE);
}

/**
	* @brief  Normal operation of MCU for application
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmDrive(void) {

	static uint8_t clockdiv = 0;
	static uint16_t a_prev = 0;
	static uint16_t b_prev = 0;
	static uint16_t c_prev = 0;

	MotorA.runPID();
	MotorB.runPID();
	MotorC.runPID();

	nucleoGeneralUpdate ngu_msg;
	ngu_msg.a_delta = MotorA.currentSpeed;
	ngu_msg.b_delta = MotorB.currentSpeed;
	ngu_msg.c_delta = MotorC.currentSpeed;

	Messaging::Message uartMsg;
	OmnibotMessaging.generateMessage(
		&uartMsg,
		(void*)&ngu_msg,
		Messaging::NUCLEO_GENERAL_UPDATE);

	OmnibotMessaging.txMessage(&uartMsg);
}

/**
	* @brief  Executes actions necessary to safely stop operation of robot
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmStopIdle(void) {
	MotorA.disarm();
	MotorB.disarm();
	MotorC.disarm();
}

/**
	* @brief  Executes actions necessary to safely respond to faults
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmFault(void) {

}


