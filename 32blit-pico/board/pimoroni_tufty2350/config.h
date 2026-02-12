#pragma once

#ifdef GITHUB_UNIVERSE
// buttons were adjusted on the final tufty for the PSRAM CS
#define BUTTON_UP_PIN   10
#define BUTTON_DOWN_PIN  6
#define BUTTON_A_PIN     7
#define BUTTON_B_PIN     8
#define BUTTON_X_PIN     9 // C
#define BUTTON_HOME_PIN 22

#else

#define BUTTON_UP_PIN   11
#define BUTTON_DOWN_PIN  6
#define BUTTON_A_PIN     7
#define BUTTON_B_PIN     9
#define BUTTON_X_PIN    10 // C
#define BUTTON_HOME_PIN 22

#endif

#define DISPLAY_ST7789

#define DBI_8BIT
#define LCD_ROTATION 270
#define LCD_CS_PIN 27
#define LCD_DC_PIN 28
#define LCD_SCK_PIN 30 // WR
#define LCD_RD_PIN 31
#define LCD_MOSI_PIN 32 // DB0
#define LCD_BACKLIGHT_PIN 26
#ifndef GITHUB_UNIVERSE
#define LCD_VSYNC_PIN 21
#endif
#define LCD_MAX_CLOCK 15000000

#define LCD_TRANSPOSE 1

#define DEFAULT_I2C_CLOCK 400000

#define LED_MONO_PINS 0, 1, 2, 3

#define USB_VENDOR_ID 0x2E8A
#define USB_PRODUCT_ID 0x1101

#define USB_VENDOR_STR "Pimoroni"
#define USB_PRODUCT_STR "Tufty 2350"
