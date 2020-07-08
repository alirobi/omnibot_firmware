/*
 * fsm.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: Karthik
 */

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

extern uint8_t spi_data[PRIMARY_SPI_BUS_DATA_SIZE_BYTES];

FSM::FSM(void) : _curState{DISABLED} {}

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
    case DRIVE:
    	if (_curState == AUX_STARTUP) {
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
void FSM::fsmDisable(void) {
	fsmTransition(CORE_STARTUP);
}
void FSM::fsmCoreStartup(void) {
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_PWM_Start(MOTOR_A_CMD1_TIMER, MOTOR_A_CMD1_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_A_CMD2_TIMER, MOTOR_A_CMD2_CHANNEL);

	HAL_TIM_PWM_Start(MOTOR_B_CMD1_TIMER, MOTOR_B_CMD1_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_B_CMD2_TIMER, MOTOR_B_CMD2_CHANNEL);

	HAL_TIM_PWM_Start(MOTOR_C_CMD1_TIMER, MOTOR_C_CMD1_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_C_CMD2_TIMER, MOTOR_C_CMD2_CHANNEL);

	fsmTransition(AUX_STARTUP);
}
void FSM::fsmConfig(void) {

}
void FSM::fsmAuxStartup(void) {
	fsmTransition(DRIVE);
}
void FSM::fsmDrive(void) {
	(&htim3)->Instance->CCR1 = _duty;
	_duty = (_duty < 500) ? 750 : 250;
}
void FSM::fsmStop(void) {

}
void FSM::fsmFault(void) {

}


