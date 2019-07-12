#ifndef __I2C_LCD_H
#define __I2C_LCD_H

#include "stm32f1xx_hal.h"

typedef struct
{
    I2C_HandleTypeDef   *I2C_Handle; //I2C handle parameters
    uint8_t             address;
    uint8_t             rows;
    uint8_t             cols;
} LCD_HandleTypeDef;

HAL_StatusTypeDef LCD_SendInternal(LCD_HandleTypeDef *lcd, uint8_t data, uint8_t flags);
void LCD_SendCommand(LCD_HandleTypeDef *lcd, uint8_t cmd);
void LCD_MoveCursor(LCD_HandleTypeDef *lcd, uint8_t line, uint8_t column);
void LCD_SendData(LCD_HandleTypeDef *lcd, uint8_t data);
void LCD_Init(LCD_HandleTypeDef *lcd);
void LCD_SendChar(LCD_HandleTypeDef *lcd,  char chr);
void LCD_SendString(LCD_HandleTypeDef *lcd,  char *str);
void LCD_MoveHome(LCD_HandleTypeDef *lcd);

#endif