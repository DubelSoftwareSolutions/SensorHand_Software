/*
 * MeasurementStruct.c
 *
 *  Created on: 02.04.2017
 *      Author: Krzysztof
 */
#include "usart.h"
#include "MeasurementStruct.h"

HAL_StatusTypeDef TransmitMeasurementsBluetooth(s_measurements p_Measurements)
{
	uint8_t i;
	uint8_t OutputData[2];
	uint16_t MessageSize;
	HAL_StatusTypeDef TransmisionStatus;
	HAL_GPIO_WritePin(LD9_GPIO_Port,LD9_Pin,GPIO_PIN_SET);

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

	HAL_GPIO_WritePin(LD9_GPIO_Port,LD9_Pin,GPIO_PIN_RESET);
	return HAL_OK;
}
