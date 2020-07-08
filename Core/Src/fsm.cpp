/*
 * fsm.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: Karthik
 */

#include "fsm.hpp"

FSM::FSM(void) : _curState{DISABLED} {}

bool FSM::fsmRun(void) {
	switch(_curState) {
	case DISABLED:
		break;
    case CORE_STARTUP:
        break;
    case CONFIG:
        break;
    case AUX_STARTUP:
		break;
    case DRIVE:
		break;
    case STOP:
		break;
    case FAULT:
        break;
    default:
        break;
	}
}

bool FSM::fsmTransition(fsmState_t nextState) {
    bool validity = INVALID_TRANS;
    switch(nextState) {
    case DISABLED:
    	if (_curState == STOP) {
    		// FSM will start in DISABLE, but otherwise should only get here
    		// after a STOP routine
			fsmDisable();
			validity = VALID_TRANS;
		}
    	break;
    case CORE_STARTUP:
        if (_curState == DISABLED) {
            fsmCoreStartup();
            validity = VALID_TRANS;
        }
        break;
    case CONFIG:
    	if (_curState == STOP) {
			// CONFIG should only occur after a STOP
			fsmConfig();
			validity = VALID_TRANS;
		}
        break;
    case AUX_STARTUP:
		if (_curState == CORE_STARTUP || _curState == CONFIG) {
			fsmAuxStartup();
			validity = VALID_TRANS;
		}
		break;
    case DRIVE:
    	if (_curState == AUX_STARTUP) {
    		// No need to call a function -- DRIVE tasks should happen on
    		// rhythm with timer
			validity = VALID_TRANS;
		}
		break;
    case STOP:
    	fsmStop();
    	validity = VALID_TRANS;
		break;
    case FAULT:
    	fsmFault();
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


