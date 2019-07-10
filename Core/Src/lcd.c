#ifndef __LCD_H
#define __LCD_H
#include "lcd.h"

#define LCD_COLS 20

#define LCD_DELAY_MS 2

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_MULTILINE 0x08
#define LCD_SINGLELINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

//moving 
const uint8_t LINE_BEGIN[4] = {
    0x00,
    0x40,
    0x14,
    0x54
};

#define LCD_PIN_EN 0b00000100  // Enable bit
#define LCD_PIN_RW 0b00000010  // Read/Write bit
#define LCD_PIN_RS 0b00000001  // Register select bit

HAL_StatusTypeDef LCD_SendInternal(LCD_HandleTypeDef *lcd, uint8_t data, uint8_t flags) {
    HAL_StatusTypeDef res;
    for(;;) {
        res = HAL_I2C_IsDeviceReady(lcd->I2C_Handle, lcd->address, 1, HAL_MAX_DELAY);
        if(res == HAL_OK)
            break;
    }

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|LCD_BACKLIGHT|LCD_PIN_EN;
    data_arr[1] = up|flags|LCD_BACKLIGHT;
    data_arr[2] = lo|flags|LCD_BACKLIGHT|LCD_PIN_EN;
    data_arr[3] = lo|flags|LCD_BACKLIGHT;

    res = HAL_I2C_Master_Transmit(lcd->I2C_Handle, lcd->address, data_arr,
                                  sizeof(data_arr), HAL_MAX_DELAY);
    HAL_Delay(LCD_DELAY_MS);
    return res;
}

void LCD_SendCommand(LCD_HandleTypeDef *lcd, uint8_t cmd) {
    LCD_SendInternal(lcd, cmd, 0);
}

void LCD_SendData(LCD_HandleTypeDef *lcd, uint8_t data) {
    LCD_SendInternal(lcd, data, LCD_PIN_RS);
}

void LCD_MoveCursor(LCD_HandleTypeDef *lcd, uint8_t line, uint8_t column) {
    if (line < lcd->rows && column < lcd->cols)
        LCD_SendCommand(lcd, LCD_SETDDRAMADDR | (LINE_BEGIN[line] + column) );
}

void LCD_Init(LCD_HandleTypeDef *lcd) {
    //http://easyelectronics.ru/avr-uchebnyj-kurs-podklyuchenie-k-avr-lcd-displeya-hd44780.html

    LCD_SendCommand(lcd, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_MULTILINE | LCD_5x8DOTS); // 0b00110000);
    // display & cursor home (keep this!)
    LCD_SendCommand(lcd, LCD_RETURNHOME);
    // display on, right shift, underline off, blink off
    LCD_SendCommand(lcd, LCD_DISPLAYCONTROL|LCD_DISPLAYON|LCD_CURSOROFF|LCD_BLINKOFF); // 0b00001100);
    // clear display (optional here)
    LCD_SendCommand(lcd, LCD_CLEARDISPLAY);
}

void LCD_SendChar(LCD_HandleTypeDef *lcd,  char chr) {
    if (chr)
        LCD_SendData(lcd, (uint8_t)(chr));
}

void LCD_SendString(LCD_HandleTypeDef *lcd,  char *str) {
    while(*str) {
        LCD_SendData(lcd, (uint8_t)(*str));
        str++;
    }
}

#endif