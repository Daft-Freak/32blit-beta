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

#ifndef SEESAW_SDA_PIN
#define SEESAW_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#endif

#ifndef SEESAW_SCL_PIN
#define SEESAW_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN
#endif

#ifndef SEESAW_ADDR
#define SEESAW_ADDR 0x50
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

enum class SeesawState {
  GPIORequest = 0,
  GPIORead,

  AnalogXRequest,
  AnalogXRead,

  AnalogYRequest,
  AnalogYRead,

  Done,
};

static SeesawState state = SeesawState::GPIORead;
static int alarm_num;

static uint32_t gpioState = ~0;
static uint16_t analogXState = 0xFF01, analogYState = 0xFF01;

static void seesaw_read(Module module, Function function, uint8_t *data, int len, int delay_us) {
  uint8_t cmd[]{uint8_t(module), uint8_t(function)};
  i2c_write_blocking(SEESAW_I2C, SEESAW_ADDR, cmd, 2, false);

  sleep_us(delay_us);

  i2c_read_blocking(SEESAW_I2C, SEESAW_ADDR, data, len, false);
}

static void seesaw_write(Module module, Function function, uint8_t *data, int len) {
  uint8_t cmd[]{uint8_t(module), uint8_t(function)};

  i2c_write_blocking(SEESAW_I2C, SEESAW_ADDR, cmd, 2, true);

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
      i2c_write_blocking(SEESAW_I2C, SEESAW_ADDR, cmd, 2, false);

      state = SeesawState::GPIORead;
      hardware_alarm_set_target(alarm_num, make_timeout_time_us(250));
      break;
    }

    case SeesawState::GPIORead: {
      i2c_read_blocking(SEESAW_I2C, SEESAW_ADDR, (uint8_t *)&gpioState, 4, false);

      state = SeesawState::AnalogXRequest;
      hardware_alarm_set_target(alarm_num, make_timeout_time_us(100));
      break;
    }

    case SeesawState::AnalogXRequest: {
      uint8_t cmd[]{uint8_t(Module::ADC), uint8_t(Function::ADC_CHANNEL14)};
      i2c_write_blocking(SEESAW_I2C, SEESAW_ADDR, cmd, 2, false);

      state = SeesawState::AnalogXRead;
      hardware_alarm_set_target(alarm_num, make_timeout_time_us(500));
      break;
    }

    case SeesawState::AnalogXRead: {
      i2c_read_blocking(SEESAW_I2C, SEESAW_ADDR, (uint8_t *)&analogXState, 2, false);

      state = SeesawState::AnalogYRequest;
      hardware_alarm_set_target(alarm_num, make_timeout_time_us(100));
      break;
    }

    case SeesawState::AnalogYRequest: {
      uint8_t cmd[]{uint8_t(Module::ADC), uint8_t(Function::ADC_CHANNEL15)};
      i2c_write_blocking(SEESAW_I2C, SEESAW_ADDR, cmd, 2, false);

      state = SeesawState::AnalogYRead;
      hardware_alarm_set_target(alarm_num, make_timeout_time_us(500));
      break;
    }

    case SeesawState::AnalogYRead: {
      i2c_read_blocking(SEESAW_I2C, SEESAW_ADDR, (uint8_t *)&analogYState, 2, false);

      state = SeesawState::Done;
      break;
    }

    case SeesawState::Done:
      break; // shouldn't happen
  }
}

void init_input() {

  // TODO: may want common i2c setup in the future?
  gpio_set_function(SEESAW_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SEESAW_SCL_PIN, GPIO_FUNC_I2C);
  i2c_init(SEESAW_I2C, 100000);

  // get product id
  uint8_t version[4]{};

  seesaw_read(Module::Status, Function::Status_VERSION, version, 4, 5);

  int product = version[0] << 8 | version[1]; // should check that this is 5743
  int day = version[2] >> 3;
  int month = (version[2] & 7) << 1 | version[3] >> 7;
  int year = version[3] & 0x7F;

  printf("Seesaw product %i date 20%02i-%02i-%02i\n", product, year, month, day);

  // init
  uint32_t io_mask = 1 << SEESAW_A_IO | 1 << SEESAW_B_IO | 1 << SEESAW_X_IO | 1 << SEESAW_Y_IO | 1 << SEESAW_START_IO | 1 << SEESAW_SELECT_IO;
  io_mask = __builtin_bswap32(io_mask); // seesaw is msb first

  // set inputs
  seesaw_write(Module::GPIO, Function::GPIO_DIRCLR, (uint8_t *)&io_mask, 4);

  // enable pullups
  seesaw_write(Module::GPIO, Function::GPIO_PULLENSET, (uint8_t *)&io_mask, 4);
  seesaw_write(Module::GPIO, Function::GPIO_SET, (uint8_t *)&io_mask, 4);

  // setup an alarm for async polling
  alarm_num = hardware_alarm_claim_unused(true);
  hardware_alarm_set_callback(alarm_num, seesaw_alarm_callback);
  hardware_alarm_set_target(alarm_num, make_timeout_time_ms(5));

  seesaw_alarm_callback(alarm_num);
}

void update_input() {
  uint32_t gpio = __builtin_bswap32(gpioState);

  uint32_t new_buttons = 0;

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
  
#else
  // joystick
  blit::api.joystick.x = (1023 - __builtin_bswap16(analogXState)) / 512.0f - 1.0f;
  blit::api.joystick.y = __builtin_bswap16(analogYState) / 512.0f - 1.0f;
#endif

  blit::api.buttons = new_buttons;

  // start new read cycle
  if(state == SeesawState::Done) {
    state = SeesawState::GPIORequest;
    seesaw_alarm_callback(alarm_num);
  }
}
