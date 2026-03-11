#pragma once

#define AUDIO_PWM_AMP_ENABLE_PIN 13

#define BUTTON_A_PIN    16
#define BUTTON_B_PIN    15
#define BUTTON_X_PIN    17
#define BUTTON_Y_PIN    18
// C/Z?
#define BUTTON_HOME_PIN 22

#define DISPLAY_ST7789

#define DBI_8BIT
#define LCD_ROTATION 90
#define LCD_CS_PIN 27
#define LCD_DC_PIN 28
#define LCD_SCK_PIN 30 // WR
#define LCD_RD_PIN 31
#define LCD_MOSI_PIN 32 // DB0
#define LCD_BACKLIGHT_PIN 26

//#define LCD_VSYNC_PIN // shared with DC
#define LCD_MAX_CLOCK 15000000

#define LCD_TRANSPOSE 1

#define DEFAULT_I2C_CLOCK 400000

#define ENABLE_CORE1 // PWM audio is otherwise unusably slow
