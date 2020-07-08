/*
 * fsm.c
 *
 *  Created on: Jul 8, 2020
 *      Author: ursk
 */

#include "fsm.h"

/*  */
void FSM_INIT(fsm_t* fsm)
{
	/* Init in the disabled state */
	fsm->state = DISABLED;

	/* Start with all the faults to be active */
	//TODO: set faults
	fsm->active_faults = 0;

	/* Choose which faults are enabled at start up */
	//TODO: set faults
	fsm->enabled_faults = 0;

}

/* Validate transition when called in fsm handlers */
int FSM_Transition(fsm_t* fsm, fsm_state_t next_state)
{
	bool valid = false;
	switch(next_state)
	{
	case DISABLED:
		if(fsm->state == STARTUP || fsm->state == FAULT || fsm->state == STOP)
		{
			valid = true;
		}
		break;
	case STARTUP:
		if(fsm->state == DISABLED)
		{
			valid = true;
		}
		break;
	case CONFIG:
		if(fsm->state == STARTUP)
		{
			valid = true;
		}
		break;
	case DRIVE:
		if(fsm->state == STARTUP)
		{
			valid = true;
		}
		break;
	case STOP:
		if(fsm->state == STARTUP)
		{
			valid = true;
		}
		break;
	case FAULT:
		if(fsm->state == ENABLED) {
			valid = true;
		}
		break;
	default:
		valid = false;
		break;
	}

	if(valid)
	{
		fsm->state = next_state;
		return true;
	}
	return false;
}

/* Retrieve active status of fault */
int getFaultStatus(fsm_t* fsm, uint32_t fault)
{
	return fsm->active_faults &= fault;
}

/* Enabling or disabling */
void enableFault(fsm_t* fsm, uint32_t fault)
{
	fsm->enabled_faults |= fault;
}

void disableFault(fsm_t* fsm, uint32_t fault)
{
	fsm->enabled_faults &= ~fault;
}

/* Set fault status to active, call fault handler */
int throwFault(fsm_t* fsm, uint32_t fault)
{
	if (~(fsm->enabled_faults &= fault))
	{
		return false;
	}

	fsm->active_faults |= fault;

	/* Set state to fault */
	fsm->state = FAULT;

	/* Through fault handler for power stage */
	/* TODO: Set power stage to brake */

	return true;
}

/* Clear active status of fault */
void clearFault(fsm_t* fsm, uint32_t fault)
{
	fsm->active_faults &= ~fault;
}

/* Main Task for motor controller
 *
 * @brief		Contains make task execution steps for
 * 				motor controller application
 */
void FSM_RUN(fsm_t* fsm)
{
	switch(fsm->state)
	{
	case DISABLED:
		MC_DISABLED_TASK();
		break;
	case STARTUP:
		MC_STARTUP_TASK();
		break;
	case ENABLED:
		MC_DRIVE_TASK();
		break;
	case STOP:
		MC_STOP_TASK();
		break;
	case FAULT:
		MC_FAULT_TASK();
		break;
	}
}
