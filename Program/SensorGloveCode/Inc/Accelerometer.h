/*
 * Accelerometer.h
 *
 *  Created on: 02.05.2017
 *      Author: Krzysztof
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "stm32f3xx_hal.h"
#include "main.h"

#define GRAVITATIONAL_ACCELERATION 9.80665

#define I2C_TIMEOUT 100
#define LSM303_ACC_ADDRESS (0x19 << 1)
#define LSM303_ACC_CTRL_REG1_A 0x20
#define LSM303_ACC_X_L_A 0x28
#define LSM303_ACC_X_H_A 0x29
#define LSM303_ACC_X_L_A_MULTI_READ (LSM303_ACC_X_L_A | 0b10000000)
#define LSM303_ACC_Y_L_A 0x2A
#define LSM303_ACC_Y_H_A 0x2B
#define LSM303_ACC_Y_L_A_MULTI_READ (LSM303_ACC_Y_L_A | 0b10000000)
#define LSM303_ACC_Z_L_A 0x2C
#define LSM303_ACC_Z_H_A 0x2D
#define LSM303_ACC_Z_L_A_MULTI_READ (LSM303_ACC_Z_L_A | 0b10000000)
#define LSM303_ACC_XYZ_MULTI_READ LSM303_ACC_X_L_A_MULTI_READ
#define LSM303_ACC_RESOLUTION_G 2
#define LSM303_ACC_RESOLUTION_MPS2 LSM303_ACC_RESOLUTION_G*GRAVITATIONAL_ACCELERATION
#define LSM303_ACC_XAXIS_ENABLE 0b00000001
#define LSM303_ACC_YAXIS_ENABLE 0b00000010
#define LSM303_ACC_ZAXIS_ENABLE 0b00000100
#define LSM303_ACC_XYZ_ENABLE 	0b00000111
#define LSM303_ACC_100HZ 0b01010000

#define ACC_MEMORY_ADD_SIZE 1
#define ACC_START_MESSAGE_SIZE 1
#define ACC_RAW_DATA_SIZE (ACCELEROMETER_AXIS_COUNT*2)

static uint8_t g_LSM303_Settings[ACC_START_MESSAGE_SIZE]=
		{LSM303_ACC_100HZ | LSM303_ACC_XYZ_ENABLE};

HAL_StatusTypeDef StartAccelerometerMeasurements();
HAL_StatusTypeDef GetAccelerometerData();

#endif /* ACCELEROMETER_H_ */