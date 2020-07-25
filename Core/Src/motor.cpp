/*
 * motor.cpp
 *
 *  Created on: Jul 15, 2020
 *      Author: Karthik
 */

#include "motor.hpp"
#include <stdlib.h>
#include <math.h>
//#include <cmath>

// TODO: design away these `extern`s

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

/**
  * @brief  Motor class constructor
  * @param  none
  * @retval none
  */
Motor::Motor( motorID_t motorID,
		float p, float i, float d,
		float samTime, float cfFreq, uint8_t dir) : 
		motorID_(motorID),
		pGain_(p), iGain_(i), dGain_(d),
		samplingTime_(samTime), cutoffFreq_(cfFreq), dir_(dir) {
	
	filtConst_ = exp(-cutoffFreq_*samplingTime_);
	remap(motorID);
}

/**
  * @brief  Raises enable pin on motor h-bridges to allow driving
  * @param  none
  * @retval Motor Status
  */
motorStatus_t Motor::arm() {
	HAL_GPIO_WritePin(armPinGPIOPort_, armPinGPIOPin_, GPIO_PIN_SET);
	return MOTOR_OK;
}

/**
  * @brief  Lowers enable pin on motor h-bridges to stop driving
  * @param  none
  * @retval Motor Status
  */
motorStatus_t Motor::disarm() {
	motorStatus_ = MOTOR_DISABLED;
	pidDisable();
	HAL_GPIO_WritePin(armPinGPIOPort_, armPinGPIOPin_, GPIO_PIN_RESET);
	return MOTOR_OK;
}

motorStatus_t Motor::getStatus() {
	return motorStatus_;
}
motorStatus_t Motor::pidEnable() {
	pidEnabled_ = true;
}
motorStatus_t Motor::pidDisable() {
	pidEnabled_ = false;

}

/**
  * @brief  Initializes Motor class and does some basic error checking
  * @param  none
  * @retval Motor Status
  */
motorStatus_t Motor::init() {
	//Check timer config
	if (cmd1TIM_->Instance->ARR == 0
		|| cmd2TIM_->Instance->ARR == 0
		|| cmd1TIM_->Instance->ARR != cmd2TIM_->Instance->ARR
		|| false) {
		motorError_ = TIM_CONFIG_ERR;
		motorStatus_ = MOTOR_ERROR;
		return MOTOR_ERROR;
	}
	cmdDutyDenom_ = cmd1TIM_->Instance->ARR;
	lastEncCount_ = (encTIM_)->Instance->CNT & 0xFFFF;
	curEncCount_ = lastEncCount_;
	// TODO: check other things
	motorStatus_ = MOTOR_OK;
	return MOTOR_OK;
}

/**
  * @brief  Finds motor command deadzone (i.e., the amount of command necessary
	*         to just start spinning the motor)
  * @param  none
  * @retval Motor Status
  */
bool Motor::calibrate() {
	if(calibrated_) return true;
	calcCurSpeed_();
	if(currentSpeed < 2) commandBase_ += 0.01;
	else {
		commandBase_ -= 0.01;
		return true;
	}
	motorCommand(commandBase_);
	return false;
}

bool Motor::calibrateToSpeed(int8_t targetSpeed) {
	if(calibrated_) return true;
	oldSpeed_ = currentSpeed;
	calcCurSpeed_();
	if (commandFFLookup_[targetSpeed] < 0.001 && targetSpeed >= 1) {
		float ref = commandFFLookup_[targetSpeed - 1] - 0.02;
		commandFFLookup_[targetSpeed] = (ref < commandFFLookup_[0] ? commandFFLookup_[0] : ref);
	}

	if (commandFFLookup_[targetSpeed] > 0.99) {
		commandFFLookup_[targetSpeed] = 1;
		return true;
	}
	else if(currentSpeed <= targetSpeed && oldSpeed_ < targetSpeed)
		commandFFLookup_[targetSpeed] += 0.005;
	else if(currentSpeed >= targetSpeed && oldSpeed_ > targetSpeed)
		commandFFLookup_[targetSpeed] -= 0.005;
	else if(currentSpeed == targetSpeed && oldSpeed_ == targetSpeed) {
		// commandFFLookup_[targetSpeed] -= 0.005;
		motorCommand(0);
		return true;
	}
	motorCommand(commandFFLookup_[targetSpeed]);
	return false;
}

void Motor::calibrateReset() {
	calibrated_ = false;
}

/**
  * @brief  Set PID controller gains
  * @param  p
  * @retval Motor Status
  */
void Motor::setPID(float p, float i, float d) {
	pGain_ = p;
	iGain_ = i;
	dGain_ = d;
}

motorStatus_t Motor::manualCommand(float cmd) {
	pidDisable();
	motorCommand(cmd);
	return MOTOR_OK;
}

void Motor::setTargetSpeed(int8_t speed) {
	if (abs(speed - targetSpeed_) > 5) iError_ = 0;
	targetSpeed_ = speed;
}

void Motor::calcCurSpeed_() {
	curEncCount_ = (encTIM_)->Instance->CNT & 0xFFFF;
	currentSpeed = static_cast<int16_t>(curEncCount_ - lastEncCount_);
}

motorStatus_t Motor::runPID() {
	if (!pidEnabled_) return MOTOR_OK;

	oldSpeed_ = currentSpeed;
	calcCurSpeed_();
	error_ = targetSpeed_ - currentSpeed;
	iError_ += error_;
	dError_ = error_ - lastError_;

	lastError_ = error_;
	lastEncCount_ = curEncCount_;

	if (targetSpeed_ == 0) command_ = 0;
	else if (targetSpeed_ > 0) {
		command_ = 
			pGain_*error_ 
			+ iGain_*iError_
			- dGain_*dError_
			+ commandFFLookup_[targetSpeed_];
	}
	else {
		command_ = 
			pGain_*error_ 
			+ iGain_*iError_
			- dGain_*dError_
			- commandFFLookup_[-targetSpeed_];
	}
	command_ = (command_ > 1) ? 1 : command_;
	command_ = (command_ < -1) ? -1 : command_;
	motorCommand(command_);

	return MOTOR_OK;
}

motorStatus_t Motor::remap(motorID_t newMotor) {
	disarm();
	motorStatus_ = MOTOR_DISABLED;

	command_ = 0;
	commandBase_ = 0;
	calibrated_ = false;
	error_ = 0;
	lastError_ = 0;
	iError_ = 0;
	dError_ = 0;

	cmdDutyDenom_ = 0;
	// TODO: Should we give the class a function pointer instead of giving it all
	// this low-level stuff?
	switch (newMotor) {
	case MOTOR_U:
		encTIM_         = MOTOR_U_ENC_TIM;
		cmd1TIM_        = MOTOR_U_CMD1_TIMER;
		cmd1TIMChannel_ = MOTOR_U_CMD1_CHANNEL;
		cmd2TIM_        = MOTOR_U_CMD2_TIMER;
		cmd2TIMChannel_ = MOTOR_U_CMD2_CHANNEL;
		armPinGPIOPort_ = MOTOR_U_ARM_GPIO_Port;
		armPinGPIOPin_  = MOTOR_U_ARM_Pin;
		break;
	case MOTOR_V:
		encTIM_         = MOTOR_V_ENC_TIM;
		cmd1TIM_        = MOTOR_V_CMD1_TIMER;
		cmd1TIMChannel_ = MOTOR_V_CMD1_CHANNEL;
		cmd2TIM_        = MOTOR_V_CMD2_TIMER;
		cmd2TIMChannel_ = MOTOR_V_CMD2_CHANNEL;
		armPinGPIOPort_ = MOTOR_V_ARM_GPIO_Port;
		armPinGPIOPin_  = MOTOR_V_ARM_Pin;
		break;
	case MOTOR_W:
		encTIM_         = MOTOR_W_ENC_TIM;
		cmd1TIM_        = MOTOR_W_CMD1_TIMER;
		cmd1TIMChannel_ = MOTOR_W_CMD1_CHANNEL;
		cmd2TIM_        = MOTOR_W_CMD2_TIMER;
		cmd2TIMChannel_ = MOTOR_W_CMD2_CHANNEL;
		armPinGPIOPort_ = MOTOR_W_ARM_GPIO_Port;
		armPinGPIOPin_  = MOTOR_W_ARM_Pin;
		break;
	default:
		// TODO
		break;
	}
}

uint16_t Motor::getEncoderCount() {
	return curEncCount_;
}

motorStatus_t Motor::writePWMDuty(TIM_HandleTypeDef * cmd_htim_ptr,
		TIMChannel_t cmd_channel, 
		float cmd_mag) {
	motorStatus_t status = MOTOR_OK;
	uint16_t duty_counts = 
	         (static_cast<uint16_t>(cmd_mag * cmdDutyDenom_)) & 0xFFFF;
	if (cmd_mag > 1.001*CMD_UPPER_LIM || cmd_mag < 0) {
		motorError_ = COMMAND_MAG_TOO_HIGH_ERR;
		duty_counts = 0;
		status = MOTOR_ERROR;
	}
	switch (cmd_channel) {
		case TIM_CHANNEL_1:
			cmd_htim_ptr->Instance->CCR1 = duty_counts;
		break;
		case TIM_CHANNEL_2:
			cmd_htim_ptr->Instance->CCR2 = duty_counts;
		break;
		case TIM_CHANNEL_3:
			cmd_htim_ptr->Instance->CCR3 = duty_counts;
		break;
		case TIM_CHANNEL_4:
			cmd_htim_ptr->Instance->CCR4 = duty_counts;
		break;
	
		default:
		// TODO
		break;
	}
	return status;
}

motorStatus_t Motor::motorCommand(float cmd) {
	if (cmd > CMD_UPPER_LIM || cmd < CMD_LOWER_LIM) {
		motorError_ = COMMAND_MAG_TOO_HIGH_ERR;
		return MOTOR_ERROR;
	}
	command_ = cmd;
	cmd *= dir_;

	motorStatus_t status;

	if (cmd >= 0) {
		status = writePWMDuty(cmd2TIM_, cmd2TIMChannel_, 0);
		status = writePWMDuty(cmd1TIM_, cmd1TIMChannel_, abs(cmd));
	}
	else {
		status = writePWMDuty(cmd1TIM_, cmd1TIMChannel_, 0);
		status = writePWMDuty(cmd2TIM_, cmd2TIMChannel_, abs(cmd));
	}

	return status;
}


