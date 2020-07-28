/*
 * fsm.hpp
 *
 *  Created on: Jul 8, 2020
 *      Author: Karthik
 */

#ifndef INC_FSM_HPP_
#define INC_FSM_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define INVALID_TRANS false
#define VALID_TRANS true

class FSM {
public:
	enum fsmState_t {
		DISABLED,
		CORE_STARTUP,
		CONFIG,
		AUX_STARTUP,
		CALIBRATE,
		DRIVE,
		STOP_IDLE,
		FAULT
	};

	FSM(void);

	bool fsmRun(void);

	bool fsmTransition(fsmState_t nextState);

	fsmState_t getCurState();

private:
	fsmState_t curState_;
	bool busy_ = false;
	bool calStage1_ = true;
	uint8_t cal_idx = 1;

	void fsmDisabled(void);
	void fsmCoreStartup(void);
	void fsmConfig(void);
	void fsmAuxStartup(void);
	void fsmCalibrate(void);
	void fsmDrive(void);
	void fsmStopIdle(void);
	void fsmFault(void);
};

#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_HPP_ */
