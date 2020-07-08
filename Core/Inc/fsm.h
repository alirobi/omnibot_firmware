#ifndef FSM_H
#define FSM_H

#include <stdbool.h>
#include <stdint.h>

//TODO: set faults

/* MOTOR Controller Moore State */
typedef enum {
	DISABLED, 			/* Controller is disabled until enabled command */
	STARTUP,			/* Controller has been requested to enable, performs startup checks */
	CONFIG,				/* Controller is enabled and responding to configuration commands */
	DRIVE,				/* Controller is ready to drive motors/read sensors/etc. */
	STOP,				/* Controller stops all operation */
	FAULT				/* Controller has thrown a fault */
} fsm_state_t;

/* Global FSM structure */
typedef struct {
	fsm_state_t state;
	uint32_t active_faults;
	uint32_t enabled_faults;
} fsm_t;

/* Initialize faults and fsm upon initialization */
void FSM_INIT(fsm_t* fsm);

/* Validate transition when called in fsm handlers */
int FSM_Transition(fsm_t* fsm, fsm_state_t next_state);

/* Retrieve active status of fault */
int getFaultStatus(fsm_t* fsm, uint32_t fault);

/* Enabling or disabling */
void enableFault(fsm_t* fsm, uint32_t fault);
void disableFault(fsm_t* fsm, uint32_t fault);

/* Set fault status to active, call fault handler */
int throwFault(fsm_t* fsm, uint32_t fault);

/* Clear active status of fault */
void clearFault(fsm_t* fsm, uint32_t fault);

void FSM_RUN(fsm_t* fsm);

#include "main.h"

#endif /* FSM_H */
