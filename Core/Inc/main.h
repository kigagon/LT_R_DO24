/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RY5_Pin GPIO_PIN_2
#define RY5_GPIO_Port GPIOE
#define RY6_Pin GPIO_PIN_3
#define RY6_GPIO_Port GPIOE
#define RY7_Pin GPIO_PIN_4
#define RY7_GPIO_Port GPIOE
#define RY8_Pin GPIO_PIN_5
#define RY8_GPIO_Port GPIOE
#define RS485_DE_Pin GPIO_PIN_13
#define RS485_DE_GPIO_Port GPIOC
#define RS485_RE_Pin GPIO_PIN_14
#define RS485_RE_GPIO_Port GPIOC
#define ADDR_DIP0_Pin GPIO_PIN_0
#define ADDR_DIP0_GPIO_Port GPIOC
#define ADDR_DIP1_Pin GPIO_PIN_1
#define ADDR_DIP1_GPIO_Port GPIOC
#define ADDR_DIP2_Pin GPIO_PIN_2
#define ADDR_DIP2_GPIO_Port GPIOC
#define ADDR_DIP3_Pin GPIO_PIN_3
#define ADDR_DIP3_GPIO_Port GPIOC
#define RUN_LED_Pin GPIO_PIN_0
#define RUN_LED_GPIO_Port GPIOA
#define ERR_LED_Pin GPIO_PIN_1
#define ERR_LED_GPIO_Port GPIOA
#define TX_LED_Pin GPIO_PIN_1
#define TX_LED_GPIO_Port GPIOB
#define RX_LED_Pin GPIO_PIN_2
#define RX_LED_GPIO_Port GPIOB
#define RY17_Pin GPIO_PIN_12
#define RY17_GPIO_Port GPIOD
#define RY18_Pin GPIO_PIN_13
#define RY18_GPIO_Port GPIOD
#define RY19_Pin GPIO_PIN_14
#define RY19_GPIO_Port GPIOD
#define RY20_Pin GPIO_PIN_15
#define RY20_GPIO_Port GPIOD
#define RY21_Pin GPIO_PIN_6
#define RY21_GPIO_Port GPIOC
#define RY22_Pin GPIO_PIN_7
#define RY22_GPIO_Port GPIOC
#define RY23_Pin GPIO_PIN_8
#define RY23_GPIO_Port GPIOC
#define RY24_Pin GPIO_PIN_9
#define RY24_GPIO_Port GPIOC
#define RY9_Pin GPIO_PIN_0
#define RY9_GPIO_Port GPIOD
#define RY10_Pin GPIO_PIN_1
#define RY10_GPIO_Port GPIOD
#define RY11_Pin GPIO_PIN_2
#define RY11_GPIO_Port GPIOD
#define RY12_Pin GPIO_PIN_3
#define RY12_GPIO_Port GPIOD
#define RY13_Pin GPIO_PIN_4
#define RY13_GPIO_Port GPIOD
#define RY14_Pin GPIO_PIN_5
#define RY14_GPIO_Port GPIOD
#define RY15_Pin GPIO_PIN_6
#define RY15_GPIO_Port GPIOD
#define RY16_Pin GPIO_PIN_7
#define RY16_GPIO_Port GPIOD
#define RY1_Pin GPIO_PIN_8
#define RY1_GPIO_Port GPIOB
#define RY2_Pin GPIO_PIN_9
#define RY2_GPIO_Port GPIOB
#define RY3_Pin GPIO_PIN_0
#define RY3_GPIO_Port GPIOE
#define RY4_Pin GPIO_PIN_1
#define RY4_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

void PY1_ON(void);
void PY1_OFF(void);
void PY2_ON(void);
void PY2_OFF(void);
void PY3_ON(void);
void PY3_OFF(void);
void PY4_ON(void);
void PY4_OFF(void);
void PY5_ON(void);
void PY5_OFF(void);
void PY6_ON(void);
void PY6_OFF(void);
void PY7_ON(void);
void PY7_OFF(void);
void PY8_ON(void);
void PY8_OFF(void);
void PY9_ON(void);
void PY9_OFF(void);
void PY10_ON(void);
void PY10_OFF(void);
void PY11_ON(void);
void PY11_OFF(void);
void PY12_ON(void);
void PY12_OFF(void);
void PY13_ON(void);
void PY13_OFF(void);
void PY14_ON(void);
void PY14_OFF(void);
void PY15_ON(void);
void PY15_OFF(void);
void PY16_ON(void);
void PY16_OFF(void);
void PY17_ON(void);
void PY17_OFF(void);
void PY18_ON(void);
void PY18_OFF(void);
void PY19_ON(void);
void PY19_OFF(void);
void PY20_ON(void);
void PY20_OFF(void);
void PY21_ON(void);
void PY21_OFF(void);
void PY22_ON(void);
void PY22_OFF(void);
void PY23_ON(void);
void PY23_OFF(void);
void PY24_ON(void);
void PY24_OFF(void);

void SW_Com(void);

void SUB_Com_SW_V(void);

#define rx2_buf_len 40
extern uint8_t Uart_rx2_buf[rx2_buf_len] ;
extern uint8_t Uart_rx2_buf_tmp[3] ;
extern int rx2_State ;
extern int rx2_buf_count;
extern int rx2_buf_count_tmp;
extern int rx2_Receive_complete;

#define tx_buf_len 14
extern uint8_t Uart_tx_buf[tx_buf_len] ;

extern uint8_t RY_Status[24] ;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
