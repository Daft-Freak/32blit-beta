#include <cstdio>

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "input.hpp"

#include "config.h"

#include "engine/api_private.hpp"
#include "engine/input.hpp"

#ifndef SEESAW_I2C
#define SEESAW_I2C i2c_default
#endif

#ifndef SEESAW_ADDR
#define SEESAW_ADDR 0x50
#endif

#ifndef SEESAW_COUNT
#define SEESAW_COUNT 1
#endif

// gamepad
#define SEESAW_A_IO 5
#define SEESAW_B_IO 1
#define SEESAW_X_IO 6
#define SEESAW_Y_IO 2
#define SEESAW_START_IO 16
#define SEESAW_SELECT_IO 0

enum class Module : uint8_t {
  Status = 0,
  GPIO = 1,
  // ...
  ADC = 9,
  // ...
};

enum class Function : uint8_t {
  Status_HW_ID   = 0x01,
  Status_VERSION = 0x02,
  Status_OPTIONS = 0x03,
  Status_TEMP    = 0x04,
  Status_SWRST   = 0x7F,

  GPIO_DIRSET    = 0x02,
  GPIO_DIRCLR    = 0x03,
  GPIO_GPIO      = 0x04,
  GPIO_SET       = 0x05,
  GPIO_CLR       = 0x06,
  GPIO_TOGGLE    = 0x07,
  GPIO_INTENSET  = 0x08,
  GPIO_INTENCLR  = 0x09,
  GPIO_INTFLAG   = 0x0A,
  GPIO_PULLENSET = 0x0B,
  GPIO_PULLENCLR = 0x0C,

  ADC_STATUS     = 0x00,
  ADC_INTENSET   = 0x02,
  ADC_INTENCLR   = 0x03,
  ADC_WINMODE    = 0x04,
  ADC_WINTHRESH  = 0x05,
  ADC_CHANNEL0   = 0x07,
  ADC_CHANNEL1   = 0x08,
  ADC_CHANNEL2   = 0x09,
  ADC_CHANNEL3   = 0x0A,
  ADC_CHANNEL4   = 0x0B,
  ADC_CHANNEL5   = 0x0C,
  ADC_CHANNEL6   = 0x0D,
  ADC_CHANNEL7   = 0x0E,
  ADC_CHANNEL8   = 0x0F,
  ADC_CHANNEL9   = 0x10,
  ADC_CHANNEL10  = 0x11,
  ADC_CHANNEL11  = 0x12,
  ADC_CHANNEL12  = 0x13,
  ADC_CHANNEL13  = 0x14,
  ADC_CHANNEL14  = 0x15,
  ADC_CHANNEL15  = 0x16,
  ADC_CHANNEL16  = 0x17,
  ADC_CHANNEL17  = 0x18,
  ADC_CHANNEL18  = 0x19,
  ADC_CHANNEL19  = 0x1A,
  ADC_CHANNEL20  = 0x1B,
};

enum class SeesawState : uint8_t {
  GPIORequest = 0,
  GPIORead,

  AnalogXRequest,
  AnalogXRead,

  AnalogYRequest,
  AnalogYRead,

  Done,
};

static SeesawState state = SeesawState::GPIORead;
static uint8_t seesaw_index = 0;
static int alarm_num;

static uint32_t gpioState[SEESAW_COUNT];
static uint16_t analogXState[SEESAW_COUNT], analogYState[SEESAW_COUNT];

static void seesaw_read(uint8_t i2c_addr, Module module, Function function, uint8_t *data, int len, int delay_us) {
  uint8_t cmd[]{uint8_t(module), uint8_t(function)};
  if(i2c_write_blocking_until(SEESAW_I2C, i2c_addr, cmd, 2, true, make_timeout_time_ms(1)) != 2)
    return;

  sleep_us(delay_us);

  i2c_read_blocking(SEESAW_I2C, i2c_addr, data, len, false);
}

static void seesaw_write(uint8_t i2c_addr, Module module, Function function, uint8_t *data, int len) {
  uint8_t cmd[]{uint8_t(module), uint8_t(function)};

  if(i2c_write_blocking_until(SEESAW_I2C, i2c_addr, cmd, 2, true, make_timeout_time_ms(1)) != 2)
    return;

  // write the rest manually
  // (we don't want a RESTART and write_raw doesn't do a STOP)
  for(int i = 0; i < len; i++) {
    while (!i2c_get_write_available(SEESAW_I2C));

    i2c_get_hw(SEESAW_I2C)->data_cmd = data[i] | (i == len - 1 ? I2C_IC_DATA_CMD_STOP_BITS : 0);
  }

  while(!(i2c_get_hw(SEESAW_I2C)->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS));

  SEESAW_I2C->restart_on_next = false;
}

static void seesaw_alarm_callback(uint alarm_num) {
  timer_hw->intr = 1 << alarm_num;

  switch(state) {
    case SeesawState::GPIORequest: {
      uint8_t cmd[]{uint8_t(Module::GPIO), uint8_t(Function::GPIO_GPIO)};
      auto timeout = make_timeout_time_us(500);

      if(i2c_write_blocking_until(SEESAW_I2C, SEESAW_ADDR + seesaw_index, cmd, 2, false, timeout) == 2)
        state = SeesawState::GPIORead;

      hardware_alarm_set_target(alarm_num, make_timeout_time_us(250));
      break;
    }

    case SeesawState::GPIORead: {
      auto timeout = make_timeout_time_us(1000);
      i2c_read_blocking_until(SEESAW_I2C, SEESAW_ADDR + seesaw_index, (uint8_t *)&gpioState[seesaw_index], 4, false, timeout);

      state = SeesawState::AnalogXRequest;
      hardware_alarm_set_target(alarm_num, make_timeout_time_us(100));
      break;
    }

    case SeesawState::AnalogXRequest: {
      uint8_t cmd[]{uint8_t(Module::ADC), uint8_t(Function::ADC_CHANNEL14)};
      auto timeout = make_timeout_time_us(500);

      if(i2c_write_blocking_until(SEESAW_I2C, SEESAW_ADDR + seesaw_index, cmd, 2, false, timeout) == 2)
        state = SeesawState::AnalogXRead;

      hardware_alarm_set_target(alarm_num, make_timeout_time_us(500));
      break;
    }

    case SeesawState::AnalogXRead: {
      auto timeout = make_timeout_time_us(1000);
      i2c_read_blocking_until(SEESAW_I2C, SEESAW_ADDR + seesaw_index, (uint8_t *)&analogXState[seesaw_index], 2, false, timeout);

      state = SeesawState::AnalogYRequest;
      hardware_alarm_set_target(alarm_num, make_timeout_time_us(100));
      break;
    }

    case SeesawState::AnalogYRequest: {
      uint8_t cmd[]{uint8_t(Module::ADC), uint8_t(Function::ADC_CHANNEL15)};
      auto timeout = make_timeout_time_us(500);

      if(i2c_write_blocking_until(SEESAW_I2C, SEESAW_ADDR + seesaw_index, cmd, 2, false, timeout) == 2)
        state = SeesawState::AnalogYRead;

      hardware_alarm_set_target(alarm_num, make_timeout_time_us(500));
      break;
    }

    case SeesawState::AnalogYRead: {
      auto timeout = make_timeout_time_us(1000);
      i2c_read_blocking_until(SEESAW_I2C, SEESAW_ADDR + seesaw_index, (uint8_t *)&analogYState[seesaw_index], 2, false, timeout);

      // stop if last device, otherwise move to the next one
      if(++seesaw_index == SEESAW_COUNT) {
        seesaw_index = 0;
        state = SeesawState::Done;
      } else {
        state = SeesawState::GPIORequest;
        hardware_alarm_set_target(alarm_num, make_timeout_time_us(100));
      }
      break;
    }

    case SeesawState::Done:
      break; // shouldn't happen
  }
}

void init_input() {
  // state
  for(auto &gpio : gpioState)
    gpio = ~0;

  for(auto &x : analogXState)
    x = 0xFF01;

  for(auto &y : analogYState)
    y = 0xFF01;

  for(int i = 0; i < SEESAW_COUNT; i++) {
    // get product id
    uint8_t version[4]{};

    seesaw_read(SEESAW_ADDR + i, Module::Status, Function::Status_VERSION, version, 4, 5);

    int product = version[0] << 8 | version[1]; // should check that this is 5743
    int day = version[2] >> 3;
    int month = (version[2] & 7) << 1 | version[3] >> 7;
    int year = version[3] & 0x7F;

    printf("Seesaw addr %02X product %i date 20%02i-%02i-%02i\n", SEESAW_ADDR + i, product, year, month, day);

    // init
    uint32_t io_mask = 1 << SEESAW_A_IO | 1 << SEESAW_B_IO | 1 << SEESAW_X_IO | 1 << SEESAW_Y_IO | 1 << SEESAW_START_IO | 1 << SEESAW_SELECT_IO;
    io_mask = __builtin_bswap32(io_mask); // seesaw is msb first

    // set inputs
    seesaw_write(SEESAW_ADDR + i, Module::GPIO, Function::GPIO_DIRCLR, (uint8_t *)&io_mask, 4);

    // enable pullups
    seesaw_write(SEESAW_ADDR + i, Module::GPIO, Function::GPIO_PULLENSET, (uint8_t *)&io_mask, 4);
    seesaw_write(SEESAW_ADDR + i, Module::GPIO, Function::GPIO_SET, (uint8_t *)&io_mask, 4);
  }

  // setup an alarm for async polling
  alarm_num = hardware_alarm_claim_unused(true);
  hardware_alarm_set_callback(alarm_num, seesaw_alarm_callback);
  hardware_alarm_set_target(alarm_num, make_timeout_time_ms(5));

  seesaw_alarm_callback(alarm_num);
}

void update_input() {
  uint32_t gpio = __builtin_bswap32(gpioState[0]);

  uint32_t new_buttons = 0;

#if SEESAW_COUNT == 1
  if(!(gpio & (1 << SEESAW_A_IO)))
    new_buttons |= blit::Button::A;

  if(!(gpio & (1 << SEESAW_B_IO)))
    new_buttons |= blit::Button::B;

  if(!(gpio & (1 << SEESAW_X_IO)))
    new_buttons |= blit::Button::X;

  if(!(gpio & (1 << SEESAW_Y_IO)))
    new_buttons |= blit::Button::Y;

  if(!(gpio & (1 << SEESAW_START_IO)))
    new_buttons |= blit::Button::HOME;

  if(!(gpio & (1 << SEESAW_SELECT_IO)))
    new_buttons |= blit::Button::MENU;

#elif SEESAW_COUNT == 2
  uint32_t gpio1 = __builtin_bswap32(gpioState[1]);
  // assumes the two devices are sideways

  if(!(gpio & (1 << SEESAW_A_IO)))
    new_buttons |= blit::Button::DPAD_DOWN;

  if(!(gpio & (1 << SEESAW_B_IO)))
    new_buttons |= blit::Button::DPAD_LEFT;

  if(!(gpio & (1 << SEESAW_X_IO)))
    new_buttons |= blit::Button::DPAD_RIGHT;

  if(!(gpio & (1 << SEESAW_Y_IO)))
    new_buttons |= blit::Button::DPAD_UP;

  if(!(gpio1 & (1 << SEESAW_A_IO)))
    new_buttons |= blit::Button::X;

  if(!(gpio1 & (1 << SEESAW_B_IO)))
    new_buttons |= blit::Button::A;

  if(!(gpio1 & (1 << SEESAW_X_IO)))
    new_buttons |= blit::Button::Y;

  if(!(gpio1 & (1 << SEESAW_Y_IO)))
    new_buttons |= blit::Button::B;

  if(!(gpio1 & (1 << SEESAW_START_IO)))
    new_buttons |= blit::Button::HOME;

  if(!(gpio1 & (1 << SEESAW_SELECT_IO)))
    new_buttons |= blit::Button::MENU;

  // don't have a joystick button and this is the only bit left
  if(!(gpio & (1 << SEESAW_START_IO)))
    new_buttons |= blit::Button::JOYSTICK;

  // there's still an unused button
#endif

#ifdef SEESAW_JOYSTICK_AS_DPAD
  int x = (1023 - __builtin_bswap16(analogXState)) - 512;
  int y = __builtin_bswap16(analogYState) - 512;

  if(x < -256)
    new_buttons |= blit::Button::DPAD_LEFT;
  else if(x > 256)
    new_buttons |= blit::Button::DPAD_RIGHT;
  if(y < -256)
    new_buttons |= blit::Button::DPAD_UP;
  else if(y > 256)
    new_buttons |= blit::Button::DPAD_DOWN;
#elif SEESAW_COUNT == 2
  blit::api_data.joystick.x = (1023 - __builtin_bswap16(analogYState[0])) / 512.0f - 1.0f;
  blit::api_data.joystick.y = (1023 - __builtin_bswap16(analogXState[0])) / 512.0f - 1.0f;
#else
  // joystick
  blit::api_data.joystick.x = (1023 - __builtin_bswap16(analogXState[0])) / 512.0f - 1.0f;
  blit::api_data.joystick.y = __builtin_bswap16(analogYState[0]) / 512.0f - 1.0f;
#endif

  blit::api_data.buttons = new_buttons;

  // start new read cycle
  if(state == SeesawState::Done) {
    state = SeesawState::GPIORequest;
    seesaw_alarm_callback(alarm_num);
  }
}
