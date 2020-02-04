/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "ethernetif.h"
#include "lwip/opt.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
#include "lwip/lwip_timers.h"
#include "ethernetif.h"
#include "inttypes.h"
#include "app_ethernet.h"
#include "stdlib.h"
#include "modbus_client.h"
#include <string.h>
#include <stdlib.h>


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define DEST_IP_ADDR0   192
#define DEST_IP_ADDR1   168
#define DEST_IP_ADDR2   1
#define DEST_IP_ADDR3   110

#define DEST_PORT       7

/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   1
#define IP_ADDR3   25

/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   1
#define GW_ADDR3   1



#define input0_Pin GPIO_PIN_14
#define input0_GPIO_Port GPIOG

#define input1_Pin GPIO_PIN_13
#define input1_GPIO_Port GPIOG

#define input2_Pin GPIO_PIN_12
#define input2_GPIO_Port GPIOG

#define input3_Pin GPIO_PIN_11
#define input3_GPIO_Port GPIOG

#define input4_Pin GPIO_PIN_10
#define input4_GPIO_Port GPIOG

#define input5_Pin GPIO_PIN_9
#define input5_GPIO_Port GPIOG

#define input6_Pin GPIO_PIN_7
#define input6_GPIO_Port GPIOD

#define input7_Pin GPIO_PIN_6
#define input7_GPIO_Port GPIOD

#define input8_Pin GPIO_PIN_5
#define input8_GPIO_Port GPIOD

#define input9_Pin GPIO_PIN_4
#define input9_GPIO_Port GPIOD

#define input10_Pin GPIO_PIN_3
#define input10_GPIO_Port GPIOD

#define input11_Pin GPIO_PIN_2
#define input11_GPIO_Port GPIOD

#define input12_Pin GPIO_PIN_1
#define input12_GPIO_Port GPIOD

#define input13_Pin GPIO_PIN_0
#define input13_GPIO_Port GPIOD

#define input14_Pin GPIO_PIN_12
#define input14_GPIO_Port GPIOC

#define input15_Pin GPIO_PIN_11
#define input15_GPIO_Port GPIOC
/* Inputs END */
/*Analog inputs*/

#define Ainput0_Pin GPIO_PIN_6
#define Ainput0_GPIO_Port GPIOF

#define Ainput1_Pin GPIO_PIN_7
#define Ainput1_GPIO_Port GPIOF

#define Ainput2_Pin GPIO_PIN_0
#define Ainput2_GPIO_Port GPIOC

#define Ainput3_Pin GPIO_PIN_1
#define Ainput3_GPIO_Port GPIOC

#define Ainput4_Pin GPIO_PIN_2
#define Ainput4_GPIO_Port GPIOC

#define Ainput5_Pin GPIO_PIN_3
#define Ainput5_GPIO_Port GPIOC



/* Outputs BEGIN */



#define output0_Pin GPIO_PIN_11
#define output0_GPIO_Port GPIOB

#define output1_Pin GPIO_PIN_10
#define output1_GPIO_Port GPIOB

#define output2_Pin GPIO_PIN_15
#define output2_GPIO_Port GPIOE

#define output3_Pin GPIO_PIN_14
#define output3_GPIO_Port GPIOE

#define output4_Pin GPIO_PIN_13
#define output4_GPIO_Port GPIOE

#define output5_Pin GPIO_PIN_12
#define output5_GPIO_Port GPIOE

#define output6_Pin GPIO_PIN_11
#define output6_GPIO_Port GPIOE

#define output7_Pin GPIO_PIN_10
#define output7_GPIO_Port GPIOE

#define output8_Pin GPIO_PIN_9
#define output8_GPIO_Port GPIOE

#define output9_Pin GPIO_PIN_8
#define output9_GPIO_Port GPIOE

#define output10_Pin GPIO_PIN_7
#define output10_GPIO_Port GPIOE

#define output11_Pin GPIO_PIN_1
#define output11_GPIO_Port GPIOG

#define output12_Pin GPIO_PIN_0
#define output12_GPIO_Port GPIOG

#define output13_Pin GPIO_PIN_15
#define output13_GPIO_Port GPIOF

#define output14_Pin GPIO_PIN_14
#define output14_GPIO_Port GPIOF

#define output15_Pin GPIO_PIN_13
#define output15_GPIO_Port GPIOF
/* Outputs END */

/*Wyjscia napieciowe */

#define Uoutput1_Pin GPIO_PIN_4
#define Uoutput1_GPIO_Port GPIOA

#define Uoutput0_Pin GPIO_PIN_5
#define Uoutput0_GPIO_Port GPIOA

#define Max_measure_vol 5
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_1
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_2
#define LED5_GPIO_Port GPIOB
#define ENC_CS_Pin GPIO_PIN_12
#define ENC_CS_GPIO_Port GPIOB
#define ENC_INT_Pin GPIO_PIN_8
#define ENC_INT_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

#define MAX_ADC_BUFFER 16

#define LED_BLUE 1
#define LED_YELLOW 2
#define LED_RED 3
#define LED_GREEN 4
#define LED_RED2 5
#define ALL_LED 6

#define LED_ON 1
#define LED_OFF 0

#define Coil_ON 0
#define Coil_OFF 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
