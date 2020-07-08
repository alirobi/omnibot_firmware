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
        DRIVE,
		STOP,
        FAULT
    };

    FSM(void);

    bool fsmRun(void);

    bool fsmTransition(fsmState_t nextState);

private:
    fsmState_t _curState;
    bool _busy = false;

    uint16_t _duty = 250;

    void fsmDisable(void);
    void fsmCoreStartup(void);
    void fsmConfig(void);
    void fsmAuxStartup(void);
    void fsmDrive(void);
    void fsmStop(void);
    void fsmFault(void);
};

#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_HPP_ */
