/*
 * fsm.hpp
 *
 *  Created on: Jul 8, 2020
 *      Author: Karthik
 */

#ifndef INC_FSM_HPP_
#define INC_FSM_HPP_

#include "main.h"
#include "stm32f4xx_hal.h"

#define INVALID_TRANS false
#define VALID_TRANS true

class FSM {
public:
	enum fsmState_t {
		DISABLED,
		STARTUP,
		CONFIG,
		DRIVE,
		FAULT
	};

	FSM(void);

	bool fsmTransition(fsmState_t nextState);

private:
	fsmState_t _curState;

	void fsmDisable(void);
	void fsmStartup(void);
	void fsmConfig(void);
	void fsmDrive(void);
	void fsmFault(void);
};

#endif /* INC_FSM_HPP_ */
