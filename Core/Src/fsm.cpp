/*
 * fsm.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: Karthik
 */

#include "fsm.hpp"

FSM::FSM(void) : _curState{DISABLED} {}

bool FSM::fsmTransition(fsmState_t nextState) {
	bool validity = INVALID_TRANS;
	switch(nextState) {
	case DISABLED:
		fsmDisable();
		_curState = nextState;
		validity = VALID_TRANS;
		break;
	case STARTUP:
		if (_curState == DISABLED) {
			fsmStartup();
			_curState = nextState;
			validity = VALID_TRANS;
		}
		break;
	case CONFIG:
		//
		break;
	case DRIVE:
		//
		break;
	case FAULT:
		//
		break;
	default:
		break;
	}
	return validity;
}



