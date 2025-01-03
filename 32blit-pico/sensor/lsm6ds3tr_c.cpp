// LSM6DS3TR-C accelerometer + gyroscope
#include "sensor.hpp"

#include "hardware/i2c.h"

#include "engine/api_private.hpp"

#ifndef LSM6DS3_ADDR
#define LSM6DS3_ADDR 0x6A
#endif

enum LSM6DS3Reg {
  CTRL1_XL = 0x10,

  STATUS_REG = 0x1E,

  OUTX_L_XL = 0x28,
  OUTX_H_XL,
  OUTY_L_XL,
  OUTY_H_XL,
  OUTZ_L_XL,
  OUTZ_H_XL,
};

void init_sensor() {
  uint8_t data[2];
  data[0] = CTRL1_XL;
  data[1] = 4 << 4; // 104Hz normal mode
  i2c_write_blocking(i2c0, LSM6DS3_ADDR, data, 2, false);
}

void update_sensor(uint32_t time) {
  uint8_t status;
  int16_t data[3];

  uint8_t reg = STATUS_REG;
  i2c_write_blocking(i2c0, LSM6DS3_ADDR, &reg, 1, true);
  i2c_read_blocking(i2c0, LSM6DS3_ADDR, &status, 1, false);

  if(status & 1) { // XLDA (new accel data)
    reg = OUTX_L_XL;
    i2c_write_blocking(i2c0, LSM6DS3_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, LSM6DS3_ADDR, (uint8_t *)&data, 6, false);

    blit::Vec3 new_tilt(-data[1], -data[0], data[2]);
    new_tilt.normalize();
    blit::api_data.tilt = new_tilt;
  }
}
