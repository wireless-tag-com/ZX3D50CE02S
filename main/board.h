#pragma once
#include "screen_driver.h"

#define INTERFACE_I2S_8

#define LCD_RESET_PIN 4
#define LCD_RS_PIN 0
#define LCD_CS_PIN -1
#define LCD_TE_PIN 48
#define LCD_WR_PIN 47
#define LCD_RD_PIN -1

#define LCD_D0_PIN 9
#define LCD_D1_PIN 46
#define LCD_D2_PIN 3
#define LCD_D3_PIN 8
#define LCD_D4_PIN 18
#define LCD_D5_PIN 17
#define LCD_D6_PIN 16
#define LCD_D7_PIN 15

#define LCD_WIDTH 320
#define LCD_HIGHT 480

//backlight pwm I/O
#define LCD_BL_PIN 45

#define TP_SDA  6
#define TP_SCL  5
#define TP_RST  4
