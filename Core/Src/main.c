/* USER CODE BEGIN Header */
/*
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal_gpio.h"
#include "lcd.h"
#include "game.h"
#include "task.h" 
#include "queue.h"
#include "music.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
  bool pressed;
  GPIO_TypeDef *port;
  uint16_t pin;
} Button;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDR (0x27 << 1)
#define LCD_COLS 20
#define LCD_ROWS 4

#define BUTTON_COUNT 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

GameEngine game;
LCD_HandleTypeDef hlcd = { 
    &hi2c1,
    LCD_ADDR,
    LCD_ROWS,
    LCD_COLS
};
char lcdBuffer[LCD_COLS+1];
Button buttons[BUTTON_COUNT] = { {
  .pressed = false,
  .port = BigButton_GPIO_Port,
  .pin = BigButton_Pin
}, {
  .pressed = false,
  .port = PlusButton_GPIO_Port,
  .pin = PlusButton_Pin
}, {
  .pressed = false,
  .port = MinusButton_GPIO_Port,
  .pin = MinusButton_Pin
} };
Button *plusButton = &buttons[1];
Button *minusButton = &buttons[2];
Button *bigButton = &buttons[0];

TimerHandle_t xSecondsTimerHandle = NULL;
TaskHandle_t xMusicHandle = NULL;
TaskHandle_t xLCDUpdaterHandle = NULL;
xSemaphoreHandle xButtonPressed;

asm(
"tetris:\n\t"
".incbin \"../../Resources/tetris.bin\"\n\t" // используем директиву .incbin
"tetris_len:\n\t"
".long .-tetris\n\t"  // вставляем значение .long с вычисленной длиной файла
);
extern uint8_t *tetris;
extern uint32_t tetris_len;
asm(
"super_mario:\n\t"
".incbin \"../../Resources/super_mario.bin\"\n\t" // используем директиву .incbin
"super_mario_len:\n\t"
".long .-super_mario\n\t"  // вставляем значение .long с вычисленной длиной файла
);
extern uint8_t *super_mario;
extern uint32_t super_mario_len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void PrintTime() {
  LCD_MoveCursor(&hlcd, 0, 0);
  sprintf(
    lcdBuffer, 
    (game.timerValue > 0 ? "        %1d:%02d        " : "       -%d:%02d        "),
    abs(game.timerValue / 60), 
    abs(game.timerValue % 60)
  );
  LCD_SendString(&hlcd, lcdBuffer);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  InitGameEngine();

	xTaskCreate(vTaskPlayerSetup, "Player", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(vTaskButtonPoll, "Buttons", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BigButton_Pin */
  GPIO_InitStruct.Pin = BigButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BigButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA8 
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB13 PB14 PB3 
                           PB4 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MinusButton_Pin PlusButton_Pin */
  GPIO_InitStruct.Pin = MinusButton_Pin|PlusButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void vTaskPlayerSetup(void *parameter)
{
  vTaskDelay(50);
  LCD_Init(&hlcd);
  vTaskDelay(50);
  LCD_MoveHome(&hlcd);
  LCD_SendString(&hlcd, "Settings");
  LCD_MoveCursor(&hlcd, 1, 0);
  while (1)
	{
    LCD_MoveCursor(&hlcd, 1, 0);
    sprintf(lcdBuffer, "Players: %1d          ", game.activePlayers);
    LCD_SendString(&hlcd, lcdBuffer);
    vTaskDelay(1);
    if (xSemaphoreTake(xButtonPressed, portMAX_DELAY) == pdPASS )
    {
      if (plusButton->pressed)
      {
        plusButton->pressed = false;
        AddPlayer();
      }
      else if (minusButton->pressed)
      {
        minusButton->pressed = false;
        RemovePlayer();
      }
      else if (bigButton->pressed && game.activePlayers > 1)
      {
        break;
      }
    }
	}
	xTaskCreate(vTaskTimerSetup, "TaskTimerSetup", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	vTaskDelete(NULL);
}

void vTaskTimerSetup(void *parameter) {
  if (xLCDUpdaterHandle != NULL)
  {
    vTaskDelete(xLCDUpdaterHandle);
    xLCDUpdaterHandle = NULL;
  }
  LCD_MoveHome(&hlcd);
  LCD_SendString(&hlcd, "Settings");
	while (1)
	{
    LCD_MoveCursor(&hlcd, 1, 0);
    sprintf(lcdBuffer, "Turn time: %1ld:%02ld     ",game.turnTime / 60, game.turnTime % 60);
    LCD_SendString(&hlcd, lcdBuffer);
    vTaskDelay(1);
    if (xSemaphoreTake(xButtonPressed, portMAX_DELAY) == pdPASS )
    {
      if (plusButton->pressed)
      {
        plusButton->pressed = false;
        IncrementTurnTime();
      }
      else if (minusButton->pressed)
      {
        minusButton->pressed = false;
        DecrementTurnTime();
      }
      else if (bigButton->pressed) {
        break;
      }
    }
	}

	xSecondsTimerHandle = xTimerCreate("SecondsTimer",
		pdMS_TO_TICKS(1000), //counts 1 sec
		pdTRUE, //auto-reload
		NULL, //not assigning ID 
		vTimerCallback // function to call after timer expires
		); 

	xTaskCreate(vTaskConfig, "Config", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	vTaskDelete(NULL);
}

void vTaskConfig(void *parameter) {
  if (xLCDUpdaterHandle != NULL)
  {
    vTaskDelete(xLCDUpdaterHandle);
    xLCDUpdaterHandle = NULL;
  }
  LCD_MoveHome(&hlcd);
  LCD_SendString(&hlcd, "Settings");
	while (1)
	{
    LCD_MoveCursor(&hlcd, 1, 0);
    sprintf(lcdBuffer, "Count scores: %3s   ", game.countScores ? "yes" : "no");
    LCD_SendString(&hlcd, lcdBuffer);
    vTaskDelay(1);
    if (xSemaphoreTake(xButtonPressed, portMAX_DELAY) == pdPASS ) {
      if (plusButton->pressed)
        game.countScores = !game.countScores;
      if (minusButton->pressed)
      // show round number or change the way it counts
        game.countScores = !game.countScores;
      if (bigButton->pressed) {
        break;
      }
      vTaskDelay(10);
    }
	}
	xTaskCreate(vTaskTurn, "TaskTurn", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	vTaskDelete(NULL);
}

void vTaskTurnTimeUpdate(void *parameter) {
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	while (1)
	{
    PrintTime();
		vTaskDelayUntil(&xLastWakeTime,1000);
  }
}

void vTaskTurn(void *parameter) {
  if (xLCDUpdaterHandle == NULL)
  {
    xTaskCreate(vTaskTurnTimeUpdate, "TimeUpdate", configMINIMAL_STACK_SIZE, NULL, 1, &xLCDUpdaterHandle);
    vTaskDelay(100);
  }
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	xTimerReset(xSecondsTimerHandle, 0);
  LCD_MoveHome(&hlcd);

  if (game.countScores)
  {
    LCD_MoveCursor(&hlcd, 2, 1);
    for (int i = 1; i <= game.activePlayers; i++)
    {
      char tmp[8];
      sprintf(tmp, "%1d:%3ld|", i, game.scores[i]);
      strcat(lcdBuffer, tmp);
      if (strlen(lcdBuffer) > (LCD_COLS-6))
      {
        LCD_SendString(&hlcd, lcdBuffer);
        LCD_MoveCursor(&hlcd, 3, 1);
      }
    }
    LCD_SendString(&hlcd, lcdBuffer);
  }

  LCD_MoveCursor(&hlcd, 1, 0);
  sprintf(lcdBuffer, "Player: %1d Score: %3ld",game.currentPlayer, game.scores[game.currentPlayer]);
  LCD_SendString(&hlcd, lcdBuffer);

  while (1)
  {
    if (xSemaphoreTake(xButtonPressed, portMAX_DELAY) == pdPASS ) 
    {
      if (plusButton->pressed) {
        xTaskCreate(vTaskTimerSetup, "TaskTimerSetup", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
        break;
      }
      else if (minusButton->pressed) {
        xTaskCreate(vTaskConfig, "Config", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
        break;
      }
      else if (bigButton->pressed) {
        xTaskCreate(vTaskTurnEnd, "TaskTurnEnd", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
        break;
      } 
    }
		vTaskDelayUntil(&xLastWakeTime,100);
//  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
	xTimerStop(xSecondsTimerHandle, 0);

	if (xMusicHandle) {
		vTaskDelete(xMusicHandle);
		MusicStop();
		xMusicHandle = NULL;
	}
	vTaskDelete(NULL);
}

void vTaskTurnEnd(void *parameter) {
  if (xLCDUpdaterHandle != NULL)
  {
    vTaskDelete(xLCDUpdaterHandle);
    xLCDUpdaterHandle = NULL;
  }
	if (game.countScores) {
		int32_t delta = 0;
    LCD_MoveHome(&hlcd);
    PrintTime();
    LCD_MoveCursor(&hlcd, 1, 0);
    sprintf(lcdBuffer, "Player: %1d Score: %3ld",game.currentPlayer, game.scores[game.currentPlayer]);
    LCD_SendString(&hlcd, lcdBuffer);
		while (1) {
      LCD_MoveCursor(&hlcd, 2, 0);
      sprintf(lcdBuffer, "Result: %+4ld", delta);
      LCD_SendString(&hlcd, lcdBuffer);
      if (xSemaphoreTake(xButtonPressed, portMAX_DELAY) == pdPASS ) {
        if (bigButton->pressed) {
          ChangeScore(delta);
          break;
        }
        else if (plusButton->pressed)
        {
          plusButton->pressed = false;
          delta++;
        }
        else if (minusButton->pressed)
        {
          minusButton->pressed = false;
          delta--;
        }
        vTaskDelay(10);
      }
    }
  }
  ResetTurnTimer();
	NextPlayer();
	xTaskCreate(vTaskTurn, "TaskTurn", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	vTaskDelete(NULL);
}

void vTaskOvertime(void *parameter) {
  /*
  Track music[2] = {
    { .begin = tetris, .size = tetris_len},
    { .begin = super_mario, .size = super_mario_len}
  };
	while (1)
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		MusicPlay(&music[0]);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		vTaskDelay(1000);
	} */
	while (1) {};
}

void vTimerCallback(TimerHandle_t xTimer) {
  game.timerValue--;
  if(game.timerValue == 0 && xMusicHandle == NULL) 
  {
    xTaskCreate(vTaskOvertime, "vTaskOvertime", configMINIMAL_STACK_SIZE, NULL, 1, &xMusicHandle);
  } 
}

void vTaskButtonPoll(void *parameter) {
  xButtonPressed = xSemaphoreCreateBinary();
  bool state = false;
  while (1) {
    for(int i = 0; i<BUTTON_COUNT; i++)
    {
      state = HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin);
      if (state != buttons[i].pressed)
      {
        vTaskDelay(25);
        state = HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin);
        if (state != buttons[i].pressed)
          buttons[i].pressed = !buttons[i].pressed;
          xSemaphoreGive(xButtonPressed);
      }
      vTaskDelay(50);
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
