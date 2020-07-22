/*
 * fsm.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: Karthik
 */

#include "fsm.hpp"
#include "motor.hpp"
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

/**
	* @brief  FSM constructor. Sets state to disabled
	* @note
	* @param  none
	* @retval none
	*/
FSM::FSM(void) : _curState{DISABLED} {}


/**
	* @brief  Runs the FSM for one step
	* @note
	* @param  none
	* @retval successful execution of FSM step
	*/
bool FSM::fsmRun(void) {
	if (_busy) return false;
	_busy = true;
	switch(_curState) {
	case DISABLED:
		fsmDisable();
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
		case STOP:
			fsmStop();
			break;
		case FAULT:
			fsmFault();
			break;
		default:
			fsmTransition(STOP);
			break;
	}
	_busy = false;
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
	switch(nextState) {
		case DISABLED:
			if (_curState == STOP) {
				// FSM will start in DISABLE, but otherwise should only get here
				// after a STOP routine
				validity = VALID_TRANS;
			}
			break;
		case CORE_STARTUP:
			if (_curState == DISABLED) {
				validity = VALID_TRANS;
			}
			break;
		case CONFIG:
			if (_curState == STOP) {
				// CONFIG should only occur after a STOP
				validity = VALID_TRANS;
			}
			break;
		case AUX_STARTUP:
			if (_curState == CORE_STARTUP || _curState == CONFIG) {
				validity = VALID_TRANS;
			}
			break;
		case CALIBRATE:
			if (_curState == AUX_STARTUP || _curState == CONFIG) {
				validity = VALID_TRANS;
			}
			break;
		case DRIVE:
			if (_curState == AUX_STARTUP || _curState == CALIBRATE) {
				validity = VALID_TRANS;
			}
			break;
		case STOP:
			validity = VALID_TRANS;
			break;
		case FAULT:
			validity = VALID_TRANS;
			break;
		default:
			break;
	}
	if (validity) {
		_curState = nextState;
	}
	return validity;
}

//TODO: Implement these properly:

/**
	* @brief  Executes actions to be taken at every execution in the DISABLED state
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmDisable(void) {
	fsmTransition(CORE_STARTUP);
}

/**
	* @brief  Executes core startup actions, such as MCU module initializations and startups
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmCoreStartup(void) {

	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_SET);

	// Start timer for timed interrupt for FSM task
	HAL_TIM_Base_Start_IT(&htim9);

	// Motor encoders ready to read
	HAL_TIM_Encoder_Start(MOTOR_U_ENC_TIM, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(MOTOR_V_ENC_TIM, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(MOTOR_W_ENC_TIM, TIM_CHANNEL_ALL);

	// Motors disarmed
	HAL_GPIO_WritePin(MOTOR_U_ARM_GPIO_Port, MOTOR_U_ARM_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_V_ARM_GPIO_Port, MOTOR_V_ARM_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_W_ARM_GPIO_Port, MOTOR_W_ARM_Pin, GPIO_PIN_RESET);

	// Motor command PWM generation -- initialized at 0% duty cycle
	HAL_TIM_PWM_Start(MOTOR_U_CMD1_TIMER, MOTOR_U_CMD1_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_U_CMD2_TIMER, MOTOR_U_CMD2_CHANNEL);

	HAL_TIM_PWM_Start(MOTOR_V_CMD1_TIMER, MOTOR_V_CMD1_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_V_CMD2_TIMER, MOTOR_V_CMD2_CHANNEL);

	HAL_TIM_PWM_Start(MOTOR_W_CMD1_TIMER, MOTOR_W_CMD1_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_W_CMD2_TIMER, MOTOR_W_CMD2_CHANNEL);

	// Auto transition
	fsmTransition(AUX_STARTUP);
}

/**
	* @brief  Configures MCU and peripherals according to new config
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmConfig(void) {

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
//
//	MotorA.setTarSpeed(-30);
//	MotorB.setTarSpeed(30);
//	MotorC.setTarSpeed(30);

	float p = 0.01; // amount of command to increase by for every count per step in error
	float i = 0.001;
	float d = 0.001;

	MotorA.setPID(p, i, d);
	MotorB.setPID(p, i, d);
	MotorC.setPID(p, i, d);
	fsmTransition(CALIBRATE);
}

void FSM::fsmCalibrate(void) {
	bool calA = MotorA.calibrate();
	bool calB = MotorB.calibrate();
	bool calC = MotorC.calibrate();

	if (calA && calB && calC) {
		MotorA.manualCommand(0);
		MotorB.manualCommand(0);
		MotorC.manualCommand(0);
		fsmTransition(DRIVE);
	}
}


/**
	* @brief  Normal operation of MCU for application
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmDrive(void) {

	MotorA.runPID();
	MotorB.runPID();
	MotorC.runPID();

}

/**
	* @brief  Executes actions necessary to safely stop operation of robot
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmStop(void) {

}

/**
	* @brief  Executes actions necessary to safely respond to faults
	* @note
	* @param  none
	* @retval none
	*/
void FSM::fsmFault(void) {

}


