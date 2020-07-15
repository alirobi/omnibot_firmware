/*
 * motor.cpp
 *
 *  Created on: Jul 15, 2020
 *      Author: Karthik
 */

#include "motor.hpp"
#include <stdlib.h> 
#include <cmath>

// TODO: design away these `extern`s

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi3;

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

Motor::Motor( motorID_t motorID,
		float p, float i, float d,
		float samTime, float cfFreq, uint8_t dir) : 
		motorID_(motorID),
		pGain_(p), iGain_(i), dGain_(d),
		samplingTime_(samTime), cutoffFreq_(cfFreq), dir_(dir) {
	
	filtConst_ = exp(-cutoffFreq_*samplingTime_);
	motorStatus_ = MOTOR_DISABLED;

	cmdDutyDenom_ = 0;

	switch (motorID_) {
	case MOTOR_A:
		encTIM_         = MOTOR_A_ENC_TIM;
		cmd1TIM_        = MOTOR_A_CMD1_TIMER;
		cmd1TIMChannel_ = MOTOR_A_CMD1_CHANNEL;
		cmd2TIM_        = MOTOR_A_CMD2_TIMER;
		cmd2TIMChannel_ = MOTOR_A_CMD2_CHANNEL;
		armPinGPIOPort_ = MOTOR_A_ARM_GPIO_Port;
		armPinGPIOPin_  = MOTOR_A_ARM_Pin;
		break;
	case MOTOR_B:
		encTIM_         = MOTOR_B_ENC_TIM;
		cmd1TIM_        = MOTOR_B_CMD1_TIMER;
		cmd1TIMChannel_ = MOTOR_B_CMD1_CHANNEL;
		cmd2TIM_        = MOTOR_B_CMD2_TIMER;
		cmd2TIMChannel_ = MOTOR_B_CMD2_CHANNEL;
		armPinGPIOPort_ = MOTOR_B_ARM_GPIO_Port;
		armPinGPIOPin_  = MOTOR_B_ARM_Pin;
		break;
	case MOTOR_C:
		encTIM_         = MOTOR_C_ENC_TIM;
		cmd1TIM_        = MOTOR_C_CMD1_TIMER;
		cmd1TIMChannel_ = MOTOR_C_CMD1_CHANNEL;
		cmd2TIM_        = MOTOR_C_CMD2_TIMER;
		cmd2TIMChannel_ = MOTOR_C_CMD2_CHANNEL;
		armPinGPIOPort_ = MOTOR_C_ARM_GPIO_Port;
		armPinGPIOPin_  = MOTOR_C_ARM_Pin;
		break;
	default:
		// TODO
		break;
	}
}

motorStatus_t Motor::arm() {
	HAL_GPIO_WritePin(armPinGPIOPort_, armPinGPIOPin_, GPIO_PIN_SET);
}

motorStatus_t Motor::disarm() {
	//
}

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
	// TODO: check other things
	motorStatus_ = MOTOR_OK;
	return MOTOR_OK;
}

void Motor::setPID(float p, float i, float d) {
	pGain_ = p;
	iGain_ = i;
	dGain_ = d;
}

motorStatus_t Motor::manualCommand(float cmd) {
	targetSpeed_ = 0;
	motorCommand(cmd);
	return MOTOR_OK;
}

void Motor::setTarSpeed(float speed) {
	targetSpeed_ = speed;
}

void Motor::calcCurSpeed(){
	// Read Encoder;
}

motorStatus_t Motor::runPID() {
//	error = targetSpeed_ - currentSpeed;
//	iError += (error + lastError) / 2 * samplingTime;
//	dError = filtConst * ((error - lastError) / samplingTime) + (1-filtConst) * dError;
//	command = pGain*error + iGain*iError + dGain*dError;
//	lastError = error;

	return MOTOR_OK;
}

motorStatus_t Motor::writePWMDuty(TIM_HandleTypeDef * cmd_htim_ptr,
		TIMChannel_t cmd_channel, 
		float duty_mag) {
	if (duty_mag > 1.001*CMD_UPPER_LIM || duty_mag < 0) {
		motorError_ = COMMAND_MAG_TOO_HIGH_ERR;
		cmd_htim_ptr->Instance->CCR1 = 0;
		return MOTOR_ERROR;
	}
	uint16_t duty_counts = (static_cast<uint16_t>(duty_mag * cmdDutyDenom_)) & 0xFFFF;
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
	return MOTOR_OK;
}

motorStatus_t Motor::motorCommand(float cmd) {
	if (cmd > CMD_UPPER_LIM || cmd < CMD_LOWER_LIM) {
		motorError_ = COMMAND_MAG_TOO_HIGH_ERR;
		return MOTOR_ERROR;
	}
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


