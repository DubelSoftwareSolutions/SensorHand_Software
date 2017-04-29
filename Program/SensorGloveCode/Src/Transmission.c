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
#include "Transmission.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		g_TransmissionReadyFlag=1;
		TransmitMeasurementsBluetooth();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart == &huart4)
		g_TransmissionReadyFlag=1;
}

HAL_StatusTypeDef StartTransmission()
{
	HAL_TIM_Base_Start_IT(&htim6);
}

HAL_StatusTypeDef TransmitFlexMeasurementsBluetooth()
{
	uint8_t i;
	uint8_t OutputData[2];
	uint16_t MessageSize;
	HAL_StatusTypeDef TransmisionStatus;
	for(i=0;i<FLEX_SENSOR_COUNT;++i)
	{
		MessageSize=sprintf(OutputData,"%d ",g_Measurements.FlexSensor[i]);
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
		MessageSize=sprintf(OutputData,"%d ",g_Measurements.TensionSensor[i]);
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
		MessageSize=sprintf(OutputData,"%d ",g_Measurements.Accelerometer[i]);
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

