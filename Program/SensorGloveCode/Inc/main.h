/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define I2C_ClockSpeed_kHz 400
#define TIM6_PRESCALER 71
#define TIM6_COUNTER_PERIOD 99
#define TIM7_PRESCALER 719
#define TIM7_COUNTER_PERIOD 99

#define DRDY_Accelerometer_Pin GPIO_PIN_2
#define DRDY_Accelerometer_GPIO_Port GPIOE
#define MEMS_INT3_Accelerometer_Pin GPIO_PIN_4
#define MEMS_INT3_Accelerometer_GPIO_Port GPIOE
#define MEMS_INT4_Accelerometer_Pin GPIO_PIN_5
#define MEMS_INT4_Accelerometer_GPIO_Port GPIOE
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define FlexSensor6_Pin GPIO_PIN_0
#define FlexSensor6_GPIO_Port GPIOC
#define FlexSensor7_Pin GPIO_PIN_1
#define FlexSensor7_GPIO_Port GPIOC
#define FlexSensor8_Pin GPIO_PIN_2
#define FlexSensor8_GPIO_Port GPIOC
#define FlexSensor9_Pin GPIO_PIN_3
#define FlexSensor9_GPIO_Port GPIOC
#define FlexSensor10_Pin GPIO_PIN_2
#define FlexSensor10_GPIO_Port GPIOF
#define BlueButton_Pin GPIO_PIN_0
#define BlueButton_GPIO_Port GPIOA
#define BlueButton_EXTI_IRQn EXTI0_IRQn
#define RubberWire1_Pin GPIO_PIN_1
#define RubberWire1_GPIO_Port GPIOA
#define RubberWire2_Pin GPIO_PIN_2
#define RubberWire2_GPIO_Port GPIOA
#define RubberWire3_Pin GPIO_PIN_3
#define RubberWire3_GPIO_Port GPIOA
#define RubberWire4_Pin GPIO_PIN_4
#define RubberWire4_GPIO_Port GPIOF
#define FlexSensor1_Pin GPIO_PIN_4
#define FlexSensor1_GPIO_Port GPIOA
#define FlexSensor2_Pin GPIO_PIN_5
#define FlexSensor2_GPIO_Port GPIOA
#define FLexSensor3_Pin GPIO_PIN_6
#define FLexSensor3_GPIO_Port GPIOA
#define FlexSensor4_Pin GPIO_PIN_7
#define FlexSensor4_GPIO_Port GPIOA
#define FlexSensor5_Pin GPIO_PIN_4
#define FlexSensor5_GPIO_Port GPIOC
#define RGB_LED_Chain_Pin GPIO_PIN_7
#define RGB_LED_Chain_GPIO_Port GPIOE
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOE
#define LD5_Pin GPIO_PIN_10
#define LD5_GPIO_Port GPIOE
#define LD7_Pin GPIO_PIN_11
#define LD7_GPIO_Port GPIOE
#define LD9_Pin GPIO_PIN_12
#define LD9_GPIO_Port GPIOE
#define LD10_Pin GPIO_PIN_13
#define LD10_GPIO_Port GPIOE
#define LD8_Pin GPIO_PIN_14
#define LD8_GPIO_Port GPIOE
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOE
#define TensionSensor1_Pin GPIO_PIN_10
#define TensionSensor1_GPIO_Port GPIOD
#define TensionSensor2_Pin GPIO_PIN_11
#define TensionSensor2_GPIO_Port GPIOD
#define TensionSensor3_Pin GPIO_PIN_12
#define TensionSensor3_GPIO_Port GPIOD
#define TensionSensor4_Pin GPIO_PIN_13
#define TensionSensor4_GPIO_Port GPIOD
#define TensionSensor5_Pin GPIO_PIN_14
#define TensionSensor5_GPIO_Port GPIOD
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Bluetooth_TX_Pin GPIO_PIN_10
#define Bluetooth_TX_GPIO_Port GPIOC
#define Bluetooth_RX_Pin GPIO_PIN_11
#define Bluetooth_RX_GPIO_Port GPIOC
#define I2C1_SCL_Accelerometer_Pin GPIO_PIN_6
#define I2C1_SCL_Accelerometer_GPIO_Port GPIOB
#define I2C1_SDA_Accelerometer_Pin GPIO_PIN_7
#define I2C1_SDA_Accelerometer_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
