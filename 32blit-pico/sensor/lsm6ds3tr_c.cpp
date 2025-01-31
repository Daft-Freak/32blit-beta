// LSM6DS3TR-C accelerometer + gyroscope
#include "sensor.hpp"

#include "hardware/i2c.h"

#include "math/constants.hpp"
#include "engine/api_private.hpp"

#ifndef LSM6DS3_ADDR
#define LSM6DS3_ADDR 0x6A
#endif

static bool lsm6ds3_present = true;

static blit::SensorDataVec3 accel_data = {nullptr, blit::SensorType::ACCELEROMETER, {}};
static blit::SensorDataVec3 gyro_data = {nullptr, blit::SensorType::GYROSCOPE, {}};

enum LSM6DS3Reg {
  CTRL1_XL = 0x10,
  CTRL2_G,

  STATUS_REG = 0x1E,

  OUTX_L_G = 0x22,
  OUTX_H_G,
  OUTY_L_G,
  OUTY_H_G,
  OUTZ_L_G,
  OUTZ_H_G,

  OUTX_L_XL = 0x28,
  OUTX_H_XL,
  OUTY_L_XL,
  OUTY_H_XL,
  OUTZ_L_XL,
  OUTZ_H_XL,
};

static void init_lsm6ds3() {
  // init accelerometer
  uint8_t data[2];
  data[0] = CTRL1_XL;
  data[1] = 4 << 4; // 104Hz normal mode
  if(i2c_write_blocking(i2c0, LSM6DS3_ADDR, data, 2, false) == PICO_ERROR_GENERIC) {
    lsm6ds3_present = false;
    return;
  }

  // init gyroscope
  data[0] = CTRL2_G;
  i2c_write_blocking(i2c0, LSM6DS3_ADDR, data, 2, false);

  blit::insert_api_sensor_data(&accel_data);
  blit::insert_api_sensor_data(&gyro_data);
}

static void update_lsm6ds3(uint32_t time) {
  if(!lsm6ds3_present)
    return;

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

    accel_data.data = new_tilt;
  }

  if(status & 2) { // GDA (new gyro data)
    reg = OUTX_L_G;
    i2c_write_blocking(i2c0, LSM6DS3_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, LSM6DS3_ADDR, (uint8_t *)&data, 6, false);

    blit::Vec3 new_gyro(-data[1], -data[0], data[2]);
    // apply scale and convert to rad/s
    new_gyro *= (245.0f/*dps*/ / 32768) * (blit::pi / 180.0f);

    gyro_data.data = new_gyro;
  }
}

SensorDriver lsm6ds3tr_c_driver {
  init_lsm6ds3, update_lsm6ds3
};
