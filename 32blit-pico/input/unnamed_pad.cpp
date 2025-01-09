#include "hardware/i2c.h"

#include "input.hpp"

#include "config.h"

#include "engine/api_private.hpp"
#include "engine/input.hpp"

// we only have two possible addresses
#define I2C_ADDR 0x55
#define I2C_ADDR2 0x57

// these are based on a vertical layout with the stick at the top
// which is a bit strange, but what the schematic uses...
/*
#define BUTTON_A_IO      2
#define BUTTON_B_IO      4
#define BUTTON_X_IO      1
#define BUTTON_Y_IO      0
*/

#ifdef DUAL_PAD_VERTICAL_EXTRA_BUTTONS
#define HAVE_SECOND
#define ROTATE_STICK

#define BUTTON_Y_IO      10
#define BUTTON_X_IO      12
#define BUTTON_B_IO      9
#define BUTTON_A_IO      8

#define BUTTON_RIGHT_IO  2
#define BUTTON_DOWN_IO   4
#define BUTTON_UP_IO     1
#define BUTTON_LEFT_IO   0

#define BUTTON_START_IO  15

#else
// horizontal layout
#define BUTTON_X_IO      2
#define BUTTON_A_IO      4
#define BUTTON_Y_IO      1
#define BUTTON_B_IO      0
#endif

// 5/6 are spare (or should be, but 5 is hacked to LED R)
#define BUTTON_STICK_IO  7

void init_input() {
}

void update_input() {
  uint8_t data[4];
  i2c_read_blocking(i2c_default, I2C_ADDR, data, 4, false);

  uint16_t raw_adc[4];
  raw_adc[0] = data[0] | (data[1] & 0xF) << 8;
  raw_adc[1] = data[2] | (data[3] & 0xF) << 8;
  uint16_t raw_buttons = data[1] >> 4 | (data[3] & 0xF0);

#ifdef HAVE_SECOND
    i2c_read_blocking(i2c_default, I2C_ADDR2, data, 4, false);
    raw_adc[2] = data[0] | (data[1] & 0xF) << 8;
    raw_adc[3] = data[2] | (data[3] & 0xF) << 8;
    raw_buttons |= (data[1] & 0xF0) << 4 | (data[3] & 0xF0) << 8;
#endif

  // process raw data
  static const int joystick_range = 1408; //1280
  static const int joystick_deadzone = 128;
  auto scale_joystick = [](uint16_t raw) {
    int val = raw - 0x800;

    val = std::min(std::max(val, -joystick_range), joystick_range);

    if(val > joystick_deadzone)
      val -= joystick_deadzone;
    else if(val < -joystick_deadzone)
      val += joystick_deadzone;
    else
      val = 0;

    return float(val) / (joystick_range - joystick_deadzone);
  };

#ifdef ROTATE_STICK
  // this looks like the non-rotated one, but we're assuming horizontal layout by default...
  blit::api_data.joystick.x =  scale_joystick(raw_adc[0]);
  blit::api_data.joystick.y =  scale_joystick(raw_adc[1]);
#else
  blit::api_data.joystick.x =  scale_joystick(raw_adc[1]);
  blit::api_data.joystick.y = -scale_joystick(raw_adc[0]);
#endif

  uint32_t new_buttons = 0;

  if(!(raw_buttons & (1 << BUTTON_A_IO)))
    new_buttons |= blit::Button::A;

  if(!(raw_buttons & (1 << BUTTON_B_IO)))
    new_buttons |= blit::Button::B;

  if(!(raw_buttons & (1 << BUTTON_X_IO)))
    new_buttons |= blit::Button::X;

  if(!(raw_buttons & (1 << BUTTON_Y_IO)))
    new_buttons |= blit::Button::Y;

// assume we either have a DPAD or we don't
#ifdef BUTTON_LEFT_IO
  if(!(raw_buttons & (1 << BUTTON_LEFT_IO)))
    new_buttons |= blit::Button::DPAD_LEFT;

  if(!(raw_buttons & (1 << BUTTON_RIGHT_IO)))
    new_buttons |= blit::Button::DPAD_RIGHT;

  if(!(raw_buttons & (1 << BUTTON_UP_IO)))
    new_buttons |= blit::Button::DPAD_UP;

  if(!(raw_buttons & (1 << BUTTON_DOWN_IO)))
    new_buttons |= blit::Button::DPAD_DOWN;
#endif

  if(!(raw_buttons & (1 << BUTTON_STICK_IO)))
    new_buttons |= blit::Button::JOYSTICK;

#ifdef BUTTON_START_IO
  if(!(raw_buttons & (1 << BUTTON_START_IO)))
    new_buttons |= blit::Button::HOME;
#endif

#ifdef DUAL_PAD_VERTICAL_EXTRA_BUTTONS
  // the last button is on one of the ADC pins
  if(raw_adc[2] == 0)
    new_buttons |= blit::Button::MENU;
#endif

  blit::api_data.buttons = new_buttons;
}
