/*
 * motor.hpp
 *
 *  Created on: Jul 15, 2020
 *      Author: Karthik
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

class Motor{
private:
	// static int motorNum = 0;
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
public:
	float currentSpeed;

	Motor(float p, float i, float d, float samTime);
	void setPID(float p, float i, float d);
	void move();
	void setTarSpeed(float speed);
	void calcCurSpeed();
	void pid();
};

#endif /* INC_MOTOR_HPP_ */
