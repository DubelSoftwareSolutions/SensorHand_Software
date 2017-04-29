/*
 * MeasurementStruct.h
 *
 *  Created on: 02.04.2017
 *      Author: Krzysztof
 */

#ifndef MEASUREMENTSTRUCT_H_
#define MEASUREMENTSTRUCT_H_

#include "stm32f3xx_hal.h"
#include "main.h"

#define FLEX_SENSOR_COUNT 10
#define TENSION_SENSOR_COUNT 5
#define ACCELEROMETER_AXIS_COUNT 3
#define SENSOR_COUNT (FLEX_SENSOR_COUNT+TENSION_SENSOR_COUNT+ACCELEROMETER_AXIS_COUNT)


typedef struct s_measurements
{
	uint16_t FlexSensor[FLEX_SENSOR_COUNT];
	uint16_t TensionSensor[TENSION_SENSOR_COUNT];
	uint16_t Accelerometer[ACCELEROMETER_AXIS_COUNT];
} s_measurements;

typedef struct s_AggregatedMeasurements
{
	float FlexSensor[FLEX_SENSOR_COUNT];
	float TensionSensor[TENSION_SENSOR_COUNT];
	float Accelerometer[ACCELEROMETER_AXIS_COUNT];
} s_AggregatedMeasurements;

#endif /* MEASUREMENTSTRUCT_H_ */
