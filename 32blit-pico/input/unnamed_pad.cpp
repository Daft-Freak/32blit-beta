#include "hardware/i2c.h"

#include "input.hpp"

#include "config.h"

#include "engine/api_private.hpp"
#include "engine/input.hpp"

#define I2C_ADDR 0x55

// these are based on a vertical layout with the stick at the top
// which is a bit strange, but what the schematic uses...
/*
#define BUTTON_A_IO      2
#define BUTTON_B_IO      4
#define BUTTON_X_IO      1
#define BUTTON_Y_IO      0
*/

// horizontal layout
#define BUTTON_X_IO      2
#define BUTTON_A_IO      4
#define BUTTON_Y_IO      1
#define BUTTON_B_IO      0

// 5/6 are spare (or should be, but 5 is hacked to LED R)
#define BUTTON_STICK_IO  7

void init_input() {
}

void update_input() {
  uint8_t data[4];
  i2c_read_blocking(i2c_default, I2C_ADDR, data, 4, false);

  uint16_t raw_adc0 = data[0] | (data[1] & 0xF) << 8;
  uint16_t raw_adc1 = data[2] | (data[3] & 0xF) << 8;
  uint8_t raw_buttons = data[1] >> 4 | (data[3] & 0xF0);

  blit::api_data.joystick.x =  float(raw_adc1 - 0x7FF) / 2047.0f;
  blit::api_data.joystick.y = -float(raw_adc0 - 0x7FF) / 2047.0f;

  uint32_t new_buttons = 0;

  if(!(raw_buttons & (1 << BUTTON_A_IO)))
    new_buttons |= blit::Button::A;

  if(!(raw_buttons & (1 << BUTTON_B_IO)))
    new_buttons |= blit::Button::B;

  if(!(raw_buttons & (1 << BUTTON_X_IO)))
    new_buttons |= blit::Button::X;

  if(!(raw_buttons & (1 << BUTTON_Y_IO)))
    new_buttons |= blit::Button::Y;

  if(!(raw_buttons & (1 << BUTTON_STICK_IO)))
    new_buttons |= blit::Button::JOYSTICK;

  blit::api_data.buttons = new_buttons;
}
