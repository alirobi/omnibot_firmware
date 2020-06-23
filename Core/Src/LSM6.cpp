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
//#include <Wire.h>
//#include <SoftWire.h>


#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS 0b1101010
#define TEST_REG_ERROR -1
#define DS33_WHO_ID 0X69

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

bool LSM6::init(deviceType device, sa0State sa0){
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
	if(_device == deviceDS33){
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
	}
}

void LSM6::writeReg(uint8_t reg, uint8_t value){
//	Wire.beginTransmission(address);
//	Wire.write(reg);
//	Wire.write(value);
//	last_status = Wire.endTransmission();

//	uint8_t data[3];
//	data[0] = reg;
//	data[1] = value>>8; //MSB byte of 16 bit data
//	data[2] = value;    //LSB byte of 16 bit data
//	HAL_I2C_Master_Transmit(&hi2c1, address, data, 3, io_timeout);
	//unsure about io_timeout and 3
//HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)

	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c1, address, (uint8_t)hi2c1.Init.OwnAddress1, reg, (uint8_t*)(&value), 3, io_timeout);
	//HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	if(status != HAL_OK){
		// err handling
	}
}

uint8_t LSM6::readReg(uint8_t reg){
//	uint8_t value;
//	Wire.beginTransmission(address);
//	Wire.write(reg);
//	last_status = Wire.endTransmission();
//	Wire.requestFrom(address, (uint8_t)1);
//	value = Wire.read();
//	Wire.endTransmission();

//	HAL_I2C_Master_Transmit(&hi2c1, address, reg, 1, 100);
//	HAL_I2C_Master_Receive(&hi2c1, address, receive_buffer, 2, 100);

//	uint8_t value = 0;
//	HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)reg, (uint8_t)1, &value, 3, io_timeout);
////	HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//	return value;

	uint8_t value;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)reg, (uint8_t)hi2c1.Init.OwnAddress1, &value, sizeof(uint16_t), io_timeout);
	//	HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	if(status == HAL_OK){
	}
	return status;
}

void LSM6::readAcc(void){
//	Wire.beginTransmission(address);
//	Wire.write(OUTX_L_XL);
//	Wire.endTransmission();
//	Wire.requestFrom(address, (uint8_t)6);
	uint8_t value = 0;
	HAL_I2C_Mem_Read(&hi2c1, address, OUTX_L_XL, (uint8_t)1, &value, (uint8_t)6, io_timeout);
	uint16_t millis_start = HAL_GetTick();
	while(hi2c1.XferCount < 6){
		if(io_timeout > 0 && ((uint16_t)HAL_GetTick() - millis_start) > io_timeout){
			did_timeout = true;
			return;
		}
	}
	uint8_t xla = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_XL, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t xha = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_XL, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t yla = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_XL, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t yha = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_XL, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t zla = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_XL, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t zha = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_XL, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);

	a.x = (int16_t)(xha << 8 | xla);
	a.y = (int16_t)(yha << 8 | yla);
	a.z = (int16_t)(zha << 8 | zla);
}

void LSM6::readGyro(void){
//	Wire.beginTransmission(address);
//	Wire.write(OUTX_L_G);
//	Wire.endTransmission();
//	Wire.requestFrom(address, (uint8_t)6);
	uint8_t value = 0;
	HAL_I2C_Mem_Read(&hi2c1, address, OUTX_L_G, (uint8_t)1, &value, (uint8_t)6, io_timeout);
	uint16_t millis_start = HAL_GetTick();
	while(hi2c1.XferCount < 6){
		if(io_timeout > 0 && ((uint16_t)HAL_GetTick() - millis_start) > io_timeout){
			did_timeout = true;
			return;
		}
	}
	uint8_t xlg = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_G, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t xhg = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_G, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t ylg = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_G, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t yhg = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_G, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t zlg = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_G, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);
	uint8_t zhg = HAL_I2C_Mem_Read(&hi2c1, address, (uint16_t)OUTX_L_G, (uint8_t)hi2c1.Init.OwnAddress1, &value, (uint16_t)1, io_timeout);

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
//int16_t LSM6::testReg(uint8_t address, regAddr reg){
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
//	}
