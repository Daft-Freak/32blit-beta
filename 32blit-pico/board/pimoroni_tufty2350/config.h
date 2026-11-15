#pragma once

#define BUTTON_UP_PIN   11
#define BUTTON_DOWN_PIN  6
#define TUFTY_BUTTON_A_PIN     7
#define TUFTY_BUTTON_B_PIN     9
#define TUFTY_BUTTON_C_PIN    10
#define BUTTON_HOME_PIN 22


#ifdef TUFTY_BUTTONS_LEFT_RIGHT
// map the lower three buttons as left, A, right
#define BUTTON_LEFT_PIN  TUFTY_BUTTON_A_PIN
#define BUTTON_A_PIN     TUFTY_BUTTON_B_PIN
#define BUTTON_RIGHT_PIN TUFTY_BUTTON_C_PIN
#else
// map the lower three buttons as A, B, X
#define BUTTON_A_PIN TUFTY_BUTTON_A_PIN
#define BUTTON_B_PIN TUFTY_BUTTON_B_PIN
#define BUTTON_X_PIN TUFTY_BUTTON_C_PIN
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
#define LCD_VSYNC_PIN 21
#define LCD_MAX_CLOCK 15000000

#define LCD_TRANSPOSE 1

#define DEFAULT_I2C_CLOCK 400000

#ifdef TUFTY_MATCH_PY_FS
#define FLASH_STORAGE_OFFSET (3 * 1024 * 1024)
#define FLASH_STORAGE_SIZE (13 * 1024 * 1024)
#endif

#define LED_MONO_PINS 0, 1, 2, 3

#define USB_VENDOR_ID 0x2E8A
#define USB_PRODUCT_ID 0x1101

#define USB_VENDOR_STR "Pimoroni"
#define USB_PRODUCT_STR "Tufty 2350"
