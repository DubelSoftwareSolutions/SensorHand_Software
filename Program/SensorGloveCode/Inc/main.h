/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
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