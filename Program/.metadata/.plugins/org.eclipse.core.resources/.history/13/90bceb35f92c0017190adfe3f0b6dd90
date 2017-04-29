/*
 * Transmission.h
 *
 *  Created on: 03.04.2017
 *      Author: Krzysztof
 */

#ifndef TRANSMISSION_H_
#define TRANSMISSION_H_

#include "stm32f3xx_hal.h"
#include "main.h"

#define TRANSMISION_TIMEOUT 1000

volatile uint8_t g_TransmissionReadyFlag;

HAL_StatusTypeDef TransmitFlexMeasurementsBluetooth()
HAL_StatusTypeDef TransmitTensionMeasurementsBluetooth();
HAL_StatusTypeDef TransmitAccelerometerMeasurementsBluetooth();

HAL_StatusTypeDef TransmitMeasurementsBluetooth(s_measurements p_Measurements);

#endif /* TRANSMISSION_H_ */
