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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "timers.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void vTaskLed(void *parameter);
void vTaskButton(void *parameter);
void vTaskDisplay(void *parameter);
void vTaskButtonPoll(void *parameter);

void vTaskBeep(void *parameter);
void vTaskOvertime(void *parameter);

void vTaskPlayerSetup(void *parameter);
void vTaskConfig(void *parameter);
void vTaskTimerSetup(void *parameter);
void vTaskTurn(void *parameter);
void vTaskTurnEnd(void *parameter);
void vTaskTurnTimeUpdate(void *parameter);

void vTimerCallback(TimerHandle_t xTimer);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOA
#define SPK1_Pin GPIO_PIN_1
#define SPK1_GPIO_Port GPIOA
#define BigButton_Pin GPIO_PIN_2
#define BigButton_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOA
#define SPK2_Pin GPIO_PIN_7
#define SPK2_GPIO_Port GPIOA
#define MinusButton_Pin GPIO_PIN_12
#define MinusButton_GPIO_Port GPIOB
#define PlusButton_Pin GPIO_PIN_15
#define PlusButton_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
