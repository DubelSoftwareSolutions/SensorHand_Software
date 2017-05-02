/*
 * Transmission.c
 *
 *  Created on: 03.04.2017
 *      Author: Krzysztof
 */

#include "usart.h"
#include "tim.h"
#include "MeasurementStruct.h"
#include "Measurements.h"
#include "DataProcessing.h"
#include "Transmission.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		if (g_Tim6iterator == g_Tim6postscaler)
		{
			AggregateMeasurementsToVoltage();
			g_TransmissionReadyFlag = 1;
			switch(g_TransmissionDevice)
			{
			case BluetoothDevice:
				TransmitMeasurementsBluetooth();
				break;
			case USBDevice:
				TransmitMeasurementsUSB();
				break;
			}
			g_Tim6iterator=0;
		}
		else
			++g_Tim6iterator;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart == &huart4)
		g_TransmissionReadyFlag=1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
		switch(g_TransmissionDevice)
		{
		case BluetoothDevice:
			g_TransmissionDevice = USBDevice;
			break;
		case USBDevice:
			g_TransmissionDevice = BluetoothDevice;
			break;
		}
}

void ConfigureTransmissionFrequency()
{
	uint32_t tim6Frequency = TIMER_CLOCK_FREQUENCY/((htim6.Init.Period+1)*(htim6.Init.Prescaler+1));
	switch(g_TransmissionDevice)
	{
	case BluetoothDevice:
		g_Tim6postscaler = tim6Frequency / BLUETOOTH_FREQUENCY;
		break;
	case USBDevice:
		g_Tim6postscaler = tim6Frequency / USB_FREQUENCY;
		break;
	}
}

HAL_StatusTypeDef StartTransmission()
{
	g_TransmissionDevice = DEFAULT_TRANSMISSION_DEVICE;
	ConfigureTransmissionFrequency();
	return HAL_TIM_Base_Start_IT(&htim6);
}

HAL_StatusTypeDef TransmitFlexMeasurementsBluetooth()
{
	uint8_t i;
	uint8_t OutputData[2];
	uint16_t MessageSize;
	HAL_StatusTypeDef TransmisionStatus;
	for(i=0;i<FLEX_SENSOR_COUNT;++i)
	{
		MessageSize=sprintf(OutputData,"%f ",g_VoltageMeasurements.FlexSensor[i]);
		while(!g_TransmissionReadyFlag);
		g_TransmissionReadyFlag =0;
		TransmisionStatus=HAL_UART_Transmit_IT(&huart4,OutputData,MessageSize);
		if(TransmisionStatus!=HAL_OK)
			return TransmisionStatus;
	}
	return HAL_OK;
}

HAL_StatusTypeDef TransmitTensionMeasurementsBluetooth()
{
	uint8_t i;
	uint8_t OutputData[2];
	uint16_t MessageSize;
	HAL_StatusTypeDef TransmisionStatus;
	for(i=0;i<TENSION_SENSOR_COUNT;++i)
	{
		MessageSize=sprintf(OutputData,"%f ",g_VoltageMeasurements.TensionSensor[i]);
		while(!g_TransmissionReadyFlag);
		g_TransmissionReadyFlag =0;
		TransmisionStatus=HAL_UART_Transmit_IT(&huart4,OutputData,MessageSize);
		if(TransmisionStatus!=HAL_OK)
			return TransmisionStatus;
	}
	return HAL_OK;
}

HAL_StatusTypeDef TransmitAccelerometerMeasurementsBluetooth()
{
	uint8_t i;
	uint8_t OutputData[2];
	uint16_t MessageSize;
	HAL_StatusTypeDef TransmisionStatus;
	for(i=0;i<ACCELEROMETER_AXIS_COUNT;++i)
	{
		MessageSize=sprintf(OutputData,"%f ",g_VoltageMeasurements.Accelerometer[i]);
		while(!g_TransmissionReadyFlag);
		g_TransmissionReadyFlag =0;
		TransmisionStatus=HAL_UART_Transmit_IT(&huart4,OutputData,MessageSize);
		if(TransmisionStatus!=HAL_OK)
			return TransmisionStatus;
	}
	return HAL_OK;
}

HAL_StatusTypeDef TransmitMeasurementsBluetooth()
{
	uint8_t OutputData[2];
	uint16_t MessageSize;
	HAL_StatusTypeDef TransmisionStatus;
	TransmitFlexMeasurementsBluetooth();
	TransmitTensionMeasurementsBluetooth();
	TransmitAccelerometerMeasurementsBluetooth();

	MessageSize=sprintf(OutputData,"\r\n");
	while(!g_TransmissionReadyFlag);
	g_TransmissionReadyFlag =0;
	TransmisionStatus=HAL_UART_Transmit_IT(&huart4,OutputData,MessageSize);
	if(TransmisionStatus!=HAL_OK)
		return TransmisionStatus;

	return HAL_OK;
}

HAL_StatusTypeDef TransmitMeasurementsUSB()
{
	//TODO
	return HAL_ERROR;
}
