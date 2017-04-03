/*
 * Transmission.c
 *
 *  Created on: 03.04.2017
 *      Author: Krzysztof
 */

#include "usart.h"
#include "MeasurementStruct.h"
#include "Transmission.h"

HAL_StatusTypeDef TransmitMeasurementsBluetooth(s_measurements p_Measurements)
{
	uint8_t i;
	uint8_t OutputData[2];
	uint16_t MessageSize;
	HAL_StatusTypeDef TransmisionStatus;

	//Sending Flex sensor measurements
	for(i=0;i<FLEX_SENSOR_COUNT;++i)
	{
		MessageSize=sprintf(OutputData,"%d ",p_Measurements.FlexSensor[i]);
		TransmisionStatus=HAL_UART_Transmit(&huart4,OutputData,MessageSize,TRANSMISION_TIMEOUT);
		if(TransmisionStatus!=HAL_OK)
			return TransmisionStatus;
	}

	//Sending Tension sensor measurements
	for(i=0;i<TENSION_SENSOR_COUNT;++i)
	{
		MessageSize=sprintf(OutputData,"%d ",p_Measurements.TensionSensor[i]);
		TransmisionStatus=HAL_UART_Transmit(&huart4,OutputData,MessageSize,TRANSMISION_TIMEOUT);
		if(TransmisionStatus!=HAL_OK)
			return TransmisionStatus;
	}

	//Sending Accelerometer measurements
	for(i=0;i<ACCELEROMETER_AXIS_COUNT;++i)
	{
		MessageSize=sprintf(OutputData,"%d ",p_Measurements.Accelerometer[i]);
		TransmisionStatus=HAL_UART_Transmit(&huart4,OutputData,MessageSize,TRANSMISION_TIMEOUT);
		if(TransmisionStatus!=HAL_OK)
			return TransmisionStatus;
	}
	MessageSize=sprintf(OutputData,"\r\n");
	TransmisionStatus=HAL_UART_Transmit(&huart4,OutputData,MessageSize,TRANSMISION_TIMEOUT);
	if(TransmisionStatus!=HAL_OK)
		return TransmisionStatus;

	return HAL_OK;
}

