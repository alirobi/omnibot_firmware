/*
 * motor.cpp
 *
 *  Created on: Jul 15, 2020
 *      Author: Karthik
 */

#include "motor.hpp"

Motor::Motor(float p, float i, float d, float samTime, float cfFreq):pGain_(p), iGain_(i), dGain_(d), samplingTime_(samTime), cutoffFreq_(cfFreq){
	filtConst_ = exp(-cutoffFreq_*samplingTime_);
}
void Motor::Motor(float p, float i, float d){
	pGain_ = p;
	iGain_ = i;
	dGain_ = d;
}
void Motor::move(){
	// Sent PWM;
}
void Motor::setTarSpeed(float speed){
	targetSpeed_ = speed;
}
void Motor::calcCurSpeed(){
	// Read Encoder;
}
void Motor::pid(){
	error = targetSpeed - currentSpeed;
	iError += (error + lastError) / 2 * samplingTime;
	dError = filtConst * ((error - lastError) / samplingTime) + (1-filtConst) * dError;
	command = pGain*error + iGain*iError + dGain*dError;
	lastError = error;
}


