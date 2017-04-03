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

HAL_StatusTypeDef TransmitMeasurementsBluetooth(s_measurements p_Measurements);

#endif /* TRANSMISSION_H_ */
