/*
 * LSM6.cpp
 *
 *  Created on: Jun 22, 2020
 *      Author: Ali
 */

#include <LSM6.hpp>
#include <math.h>
#include <stm32f4xx_hal_i2c_ex.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_def.h>
#include <main.h>
//#include <Wire.h>
//#include <SoftWire.h>


#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS 0b1101010
#define TEST_REG_ERROR -1
#define DS33_WHO_ID 0X69
#define LSM6_ADDRESS 0b1101011

LSM6::LSM6(void){
	_device = deviceAuto;
	io_timeout = 0;
	did_timeout = false;
}

bool LSM6::timeoutOccured(){
	bool tmp = did_timeout;
	did_timeout = false;
	return tmp;
}

void LSM6::setTimeout(uint16_t timeout){
	io_timeout = timeout;
}

uint16_t LSM6::getTimeout(){
	return io_timeout;
}

bool LSM6::init(deviceType device, sa0State sa0, I2C_HandleTypeDef * hi2c_ptr_in){
	hi2c_ptr = hi2c_ptr_in;
	if(device == deviceAuto || sa0 == sa0Auto){
		if(device == deviceAuto || device == deviceDS33){
			if(sa0 != sa0Low && testReg(DS33_SA0_HIGH_ADDRESS, WHO_AM_I) == DS33_WHO_ID){
				sa0 = sa0High;
				if(device == deviceAuto){
					device = deviceDS33;
				}
			}
			else if(sa0 != sa0High && testReg(DS33_SA0_LOW_ADDRESS, WHO_AM_I) == DS33_WHO_ID){
				sa0 = sa0Low;
				if(device == deviceAuto){
					device = deviceDS33;
				}
			}
		}
		if(device == deviceAuto || sa0 == sa0Auto){
			return false;
		}
	}
	_device = device;
	switch(device){
		case deviceDS33:
			address = (sa0 == sa0High) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
			break;
		case deviceAuto:
			break;
	}
	return true;
}

void LSM6::enableDefault(void){
//	__HAL_DBGMCU_FREEZE_I2C1_TIMEOUT();
//	if(_device == deviceDS33){
		//Accelerometer
		// 0x80 = 0b10000000
		// ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
		writeReg(CTRL1_XL, 0x80);
		//Gyro
		// 0x80 = 0b010000000
		// ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
		writeReg(CTRL2_G, 0x80);
		//Common
		// 0x04 = 0b00000100
		// IF_INC = 1 (automatically increment register address)
		writeReg(CTRL3_C, 0x04);
//	}
//		__HAL_DBGMCU_UNFREEZE_I2C1_TIMEOUT();
}

void LSM6::writeReg(uint8_t reg, uint8_t value){

	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(hi2c_ptr, (uint8_t)(LSM6_ADDRESS<<1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(&value), 1, 100);
}

uint8_t LSM6::readReg(uint8_t reg, uint8_t* buff){

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c_ptr, (uint8_t)(LSM6_ADDRESS<<1), reg, I2C_MEMADD_SIZE_8BIT, buff, 1, 100);
	if(status == HAL_OK){
	}
	return buff[0];
}

void LSM6::readAcc(void){
//	Wire.beginTransmission(address);
//	Wire.write(OUTX_L_XL);
//	Wire.endTransmission();
//	Wire.requestFrom(address, (uint8_t)6);
	uint8_t buff[6] = {0};
//	uint8_t *p = buff;
//	*p = OUTX_L_XL;
	buff[1] = OUTX_H_XL;
	buff[2] = OUTY_L_XL;
	buff[3] = OUTY_H_XL;
	buff[4] = OUTZ_L_XL;
	buff[5] = OUTZ_H_XL;
	HAL_I2C_Master_Transmit(hi2c_ptr, LSM6_ADDRESS << 1, buff, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c_ptr, LSM6_ADDRESS << 1, buff, 6, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(hi2c_ptr, address, OUTX_L_XL, (uint8_t)1, &value, (uint8_t)6, io_timeout);
	uint16_t millis_start = HAL_GetTick();
	while(hi2c_ptr->XferSize < 6){
		if(io_timeout > 0 && ((uint16_t)HAL_GetTick() - millis_start) > io_timeout){
			did_timeout = true;
			return;
		}
	}
	uint8_t xla = buff[0];
	uint8_t xha = buff[1];
	uint8_t yla = buff[2];
	uint8_t yha = buff[3];
	uint8_t zla = buff[4];
	uint8_t zha = buff[5];

	a.x = (int16_t)(xha << 8 | xla);
	a.y = (int16_t)(yha << 8 | yla);
	a.z = (int16_t)(zha << 8 | zla);
}

void LSM6::readGyro(void){
//	Wire.beginTransmission(address);
//	Wire.write(OUTX_L_G);
//	Wire.endTransmission();
//	Wire.requestFrom(address, (uint8_t)6);
	uint8_t buff2[6];
	uint8_t *p2 = buff2;
	*p2 = OUTX_L_G;
//	buff2[0] = OUTX_L_G;
//	buff2[1] = OUTX_H_G;
//	buff2[2] = OUTY_L_G;
//	buff2[3] = OUTY_H_G;
//	buff2[4] = OUTZ_L_G;
//	buff2[5] = OUTZ_H_G;
	HAL_I2C_Master_Transmit(hi2c_ptr, LSM6_ADDRESS , buff2, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c_ptr, LSM6_ADDRESS , buff2, 6, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(hi2c_ptr, address, OUTX_L_G, (uint8_t)1, &value, (uint8_t)6, io_timeout);
	uint16_t millis_start = HAL_GetTick();
	while(hi2c_ptr->XferSize < 6){
		if(io_timeout > 0 && ((uint16_t)HAL_GetTick() - millis_start) > io_timeout){
			did_timeout = true;
			return;
		}
	}
	uint8_t xlg = buff2[0];
	uint8_t xhg = buff2[1];
	uint8_t ylg = buff2[2];
	uint8_t yhg = buff2[3];
	uint8_t zlg = buff2[4];
	uint8_t zhg = buff2[5];

	g.x = (int16_t)(xhg << 8 | xlg);
	g.y = (int16_t)(yhg << 8 | ylg);
	g.z = (int16_t)(zhg << 8 | zlg);
}
void LSM6::read(void){
	readAcc();
	readGyro();
}

void LSM6::vector_cross(const vec *a, const vec *b, vec *out){
		out-> x = (a->y * b->z) - (a->z * b->y);
		out-> y = (a->z * b->x) - (a->x * b->z);
		out-> z = (a->x * b->y) - (a->y * b->x);
	}

float LSM6::vector_dot(const vec *a, const vec *b){
	return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void LSM6::vectorNormalize(vec *a){
	float mag = sqrt(vector_dot(a, a));
	a->x /= mag;
	a->y /= mag;
	a->z /= mag;
}

//dont think this is doable
int16_t LSM6::testReg(uint8_t address, regAddr reg){
//	Wire.beginTransmission(address);
//	Wire.write((uint8_t)reg);
//	if(Wire.endTransmission() != 0){
//		return TEST_REG_ERROR;
//	}
//	Wire.requestFrom(address, (uint8_t)1);
//	if(Wire.available()){
//		return Wire.read();
//	}else{
//		return TEST_REG_ERROR;
	}
