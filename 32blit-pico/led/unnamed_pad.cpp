#include "led.hpp"

#include "hardware/i2c.h"

#include "engine/api_private.hpp"

#define I2C_ADDR 0x55

blit::Pen last_led;

void init_led() {
}

void update_led() {
  if(!(blit::api_data.LED == last_led)) {
    last_led = blit::api_data.LED;
    i2c_write_blocking(i2c0, I2C_ADDR, &last_led.r, 3, false);
  }
}
