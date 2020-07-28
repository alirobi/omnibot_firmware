
/*
 * application.cpp
 *
 *  Created on: Jun 29, 2020
 *      Author: Karthik
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "application.hpp"
#include "lsm6.hpp"
#include "fsm.hpp"
#include "motor.hpp"
#include "messaging.hpp"
#include "main.h"

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
extern uint8_t* const sdata_head;

void sendMessageUART(Messaging::Message* msg_buf);

void rxMsgCallback(Messaging::Message &msg);

LSM6      IMU;
Motor     MotorA(MOTOR_U, 0.005, 0.0005, 0.0005, 0.01, 100, DIR_DEFAULT);
Motor     MotorB(MOTOR_V, 0.005, 0.0005, 0.0005, 0.01, 100, DIR_DEFAULT);
Motor     MotorC(MOTOR_W, 0.005, 0.0005, 0.0005, 0.01, 100, DIR_DEFAULT);
FSM       OmnibotFSM;
Messaging OmnibotMessaging(&sendMessageUART, &rxMsgCallback);

/**
  * @brief  Executes general startup actions, such as initializing classes
  * @note	NOT for use in initializing internal MCU modules \ 
	*	@note See @ref FSM::coreStartup()
  * @param  none
  * @retval none
  */
void setup(void) {

	// run what's defined in fsmDisabled
	OmnibotFSM.fsmRun();
	// transition to coreStartup

	// run what's defined in fsmCoreStartup
	OmnibotFSM.fsmRun();
	// transition to stop idle
	// timer has started so now FSM will just run. It won't be doing anything
	// without another stateTransition

	velocityCmd velocityCmd_msg;
	uint8_t size1 = sizeof(velocityCmd_msg);
	manualCmd manualCmd_msg;
	uint8_t size2 = sizeof(manualCmd_msg);
	heartbeat heartbeat_msg;
	uint8_t size3 = sizeof(heartbeat_msg);
	motorRemap motorRemap_msg;
	uint8_t size4 = sizeof(motorRemap_msg);
	globalPID globalPID_msg;
	uint8_t size5 = sizeof(globalPID_msg);
	singlePID singlePID_msg;
	uint8_t size6 = sizeof(singlePID_msg);
	nucleoGeneralUpdate nucleoGeneralUpdate_msg;
	uint8_t size7 = sizeof(nucleoGeneralUpdate_msg);

//	IMU.init(LSM6::deviceAuto, LSM6::sa0Auto, &hi2c1);
//	IMU.enableDefault();

	MotorA.init();
	MotorB.init();
	MotorC.init();
}

/**
  * @brief  Executes actions on an un-timed loop
  * @note
  * @param  none
  * @retval none
  */
void loop(void) {
//	IMU.readAcc();
//	IMU.readGyro();
}

/**
  * @brief  Called from Interrupt Service Routines (ISRs)
  * 		running from @ref /Core/Src/stm32f4xx_it.c
  * @note	Expand as necessary
  * @param  none
  * @retval none
  */
void interruptLink(interruptLink_t it) {
	switch(it) {
		case TIM9_IT:
			// PRIMARY FSM TASK
			OmnibotFSM.fsmRun();
			break;
		default:
			break;
	}
}

void dmaLink(dmaLink_t dma) {
	switch(dma) {
		case UART1_DMA:
			bool success = OmnibotMessaging.rxMessageSequence((uint8_t*)sdata);
			break;
//		default:
//			return;
//			break;
	}
	return;
}

void sendMessageUART(Messaging::Message* msg_buf) {
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, SDATA_SIZE_BYTES, 0xFFFF);
	return;
}

void rxMsgCallback(Messaging::Message &msg) {

	switch(msg.msgType) {
		case Messaging::VELOCITY_CMD: {
			MotorA.pidEnable();
			MotorB.pidEnable();
			MotorC.pidEnable();

			velocityCmd velocityCmd_data = *((velocityCmd*)(msg.msgData));
			
			MotorA.setTargetSpeed(velocityCmd_data.vel_a);
			MotorB.setTargetSpeed(velocityCmd_data.vel_b);
			MotorC.setTargetSpeed(velocityCmd_data.vel_c);

			if (!OmnibotFSM.fsmTransition(FSM::DRIVE)) {
				if (!OmnibotFSM.fsmTransition(FSM::AUX_STARTUP)) {
					if (!OmnibotFSM.fsmTransition(FSM::AUX_STARTUP)) {
						while (1) {}
					}
				}
			}
		}break;
		case Messaging::MANUAL_CMD: {
			MotorA.pidDisable();
			MotorB.pidDisable();
			MotorC.pidDisable();

			manualCmd manualCmd_data = *((manualCmd*)(msg.msgData));

			MotorA.manualCommand(manualCmd_data.cmd_a);
			MotorB.manualCommand(manualCmd_data.cmd_b);
			MotorC.manualCommand(manualCmd_data.cmd_c);

			OmnibotFSM.fsmTransition(FSM::DRIVE);
		}break;
		case Messaging::HEARTBEAT: {
			heartbeat heartbeat_data = *((heartbeat*)(msg.msgData));
			if (!heartbeat_data.beat) OmnibotFSM.fsmTransition(FSM::STOP_IDLE);
		}break;
		case Messaging::STATE_TRANSITION: {
		}break;
		case Messaging::MOTOR_REMAP: {
		}break;
		case Messaging::GLOBAL_PID: {
			OmnibotFSM.fsmTransition(FSM::STOP_IDLE);
			OmnibotFSM.fsmTransition(FSM::CONFIG);

			globalPID * globalPID_data = ((globalPID*)(msg.msgData));

			float p = globalPID_data->pGain;
			float i = globalPID_data->iGain;
			float d = globalPID_data->dGain;

			MotorA.setPID(p, i, d);
			MotorB.setPID(p, i, d);
			MotorC.setPID(p, i, d);
			OmnibotFSM.fsmTransition(FSM::STOP_IDLE);
		}break;
		case Messaging::SINGLE_PID: {
			OmnibotFSM.fsmTransition(FSM::STOP_IDLE);
			OmnibotFSM.fsmTransition(FSM::CONFIG);

			singlePID singlePID_data = *((singlePID*)(msg.msgData));

			switch (singlePID_data.motorID) {
				case 0:
					MotorA.setPID(singlePID_data.pGain,
			                  singlePID_data.iGain,
			                  singlePID_data.dGain);
					break;
				case 1:
					MotorA.setPID(singlePID_data.pGain,
			                  singlePID_data.iGain,
			                  singlePID_data.dGain);
					break;
				case 2:
					MotorA.setPID(singlePID_data.pGain,
			                  singlePID_data.iGain,
			                  singlePID_data.dGain);
					break;
			}
			OmnibotFSM.fsmTransition(FSM::STOP_IDLE);
		}break;
		case Messaging::NULLMSG: {
		}break;
		default: {
		}break;
	}
	return;
}

#ifdef __cplusplus
}
#endif
