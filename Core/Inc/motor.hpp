/*
 * motor.hpp
 *
 *  Created on: Jul 15, 2020
 *      Author: Karthik
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#define TIMChannel_t uint32_t
#define DIR_DEFAULT 1
#define DIR_FLIPPED -1
#define CMD_UPPER_LIM 1
#define CMD_LOWER_LIM -1

#include "main.h"
#include "stm32f4xx_hal.h"

enum motorID_t {
	MOTOR_A,
	MOTOR_B,
	MOTOR_C
};

enum motorStatus_t {
	MOTOR_DISABLED,
	MOTOR_OK,
	MOTOR_ERROR
};

enum motorError_t {
	NO_ERR,
	MOTOR_CONFIG_ERR,
	TIM_CONFIG_ERR,
	COMMAND_MAG_TOO_HIGH_ERR
};

class Motor{
public:
	float currentSpeed;

	Motor(motorID_t motorID, float p, float i, float d, float samTime, float cfFreq, uint8_t dir);
	motorStatus_t init();
	motorStatus_t arm();
	motorStatus_t disarm();
	void setPID(float p, float i, float d);
	motorStatus_t manualCommand(float cmd);
	void setTarSpeed(float speed);
	void calcCurSpeed();
	motorStatus_t runPID();

private:
	motorID_t motorID_;
	motorStatus_t motorStatus_;
	motorError_t motorError_;

	float pGain_, iGain_, dGain_;
	float samplingTime_;
	float targetSpeed_;
	float command_;
	float error_;
	float lastError_;
	float iError_;
	float dError_;
	float cutoffFreq_;
	float filtConst_;
	uint8_t dir_;
	uint16_t cmdDutyDenom_;
	
	TIM_HandleTypeDef * encTIM_;
	TIM_HandleTypeDef * cmd1TIM_;
	TIMChannel_t cmd1TIMChannel_;
	TIM_HandleTypeDef * cmd2TIM_;
	TIMChannel_t cmd2TIMChannel_;
	GPIO_TypeDef * armPinGPIOPort_;
	uint16_t armPinGPIOPin_;

	motorStatus_t writePWMDuty(TIM_HandleTypeDef * cmd_htim_ptr,
			TIMChannel_t cmd_channel,
			float duty_mag);
	motorStatus_t motorCommand(float cmd);
};

#endif /* INC_MOTOR_HPP_ */
