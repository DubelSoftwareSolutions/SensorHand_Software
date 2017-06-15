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
#define TRANSMISSION_DATA_SIZE 10
#define TIMER_CLOCK_FREQUENCY 72000000
#define BLUETOOTH_FREQUENCY 20 //Hz
#define UART_SERIAL_FREQUENCY 280 //Hz
#define USB_FREQUENCY 1000 //Hz
#define DEFAULT_TRANSMISSION_DEVICE BluetoothDevice

volatile uint8_t g_TransmissionReadyFlag;
volatile uint8_t g_TransmissionCpltFlag;
volatile uint32_t g_Tim6postscaler;
volatile uint32_t g_Tim6iterator;

void ConfigureTransmissionFrequency();
HAL_StatusTypeDef StartTransmission();
HAL_StatusTypeDef ContinueTransmission();

HAL_StatusTypeDef TransmitFlexMeasurementsBluetooth();
HAL_StatusTypeDef TransmitTensionMeasurementsBluetooth();
HAL_StatusTypeDef TransmitAccelerometerMeasurementsBluetooth();

HAL_StatusTypeDef TransmitMeasurementsBluetooth();

USBD_StatusTypeDef TransmitFlexMeasurementsUSB();
USBD_StatusTypeDef TransmitTensionMeasurementsUSB();
USBD_StatusTypeDef TransmitAccelerometerMeasurementsUSB();

USBD_StatusTypeDef TransmitMeasurementsUSB();

#endif /* TRANSMISSION_H_ */
