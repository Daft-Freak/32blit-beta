// BME280 temperature + pressure + humidity
#include "sensor.hpp"

#include "hardware/i2c.h"

#include "math/constants.hpp"
#include "engine/api_private.hpp"

#ifndef BME280_ADDR
#define BME280_ADDR 0x76
#endif

static bool bme280_present = true;

static blit::SensorDataFloat press_data(blit::SensorType::PRESSURE);
static blit::SensorDataFloat temp_data(blit::SensorType::TEMPERATURE);
static blit::SensorDataFloat hum_data(blit::SensorType::HUMIDITY);

enum BME280Reg {
  CALIB00 = 0x88, // 88-A1
  CALIB25 = 0xA1,
  CALIB26 = 0xE1, // E1-E7

  CTRL_HUM = 0xF2,
  CTRL_MEAS = 0xF4,
  CONFIG = 0xF5,

  PRESS_MSB = 0xF7,
  PRESS_LSB,
  PRESS_XLSB,

  TEMP_MSB = 0xFA,
  TEMP_LSB,
  TEMP_XLSB,

  HUM_MSB = 0xFD,
  HUM_LSB,
};

struct BME280Calibration {
  uint16_t dig_T1;
  int16_t dig_T2, dig_T3;

  uint16_t dig_P1;
  int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

  uint8_t dig_H1, dig_H3;
  int16_t dig_H2, dig_H4, dig_H5;
  int8_t dig_H6;
};

static BME280Calibration calib;

// compensation functions from the datasheet, slightly reformatted

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
static int32_t t_fine;
static int32_t bme280_compensate_T_int32(int32_t adc_T) {
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bme280_compensate_P_int64(int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
  if (var1 == 0)
    return 0; // avoid exception caused by division by zero

  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
  return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
static uint32_t bme280_compensate_H_int32(int32_t adc_H) {
  int32_t v_x1_u32r;
  v_x1_u32r = t_fine - 76800;
  v_x1_u32r = ((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) + (16384)) >> 15)
            * (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + (32768))) >> 10) + (2097152)) * ((int32_t)calib.dig_H2) + 8192) >> 14);
  v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4);
  v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
  v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
  return (uint32_t)v_x1_u32r >> 12;
}

static void init_bme280() {
  // init
  // these are the "gaming" settings, which disable humidity... hmm
  uint8_t data[7];
  data[0] = CTRL_HUM;
  data[1] = 0; // oversample x0
  if(i2c_write_blocking(i2c0, BME280_ADDR, data, 2, false) == PICO_ERROR_GENERIC) {
    bme280_present = false;
    return;
  }

  data[0] = CTRL_MEAS;
  data[1] = (1 /*temp oversample x1*/ << 5) | (3 /*pressure oversample x4*/ << 2) | 3 /*normal mode*/;
  i2c_write_blocking(i2c0, BME280_ADDR, data, 2, false);

  data[0] = CONFIG;
  data[1] = (4 /*filter coeff 16*/ << 2);
  i2c_write_blocking(i2c0, BME280_ADDR, data, 2, false);

  // read calibration
  // first chunk should align with our struct
  data[0] = CALIB00;
  i2c_write_blocking(i2c0, BME280_ADDR, data, 1, true);
  i2c_read_blocking(i2c0, BME280_ADDR, (uint8_t *)&calib, 24, false);

  // humidity values are a bit messy
  data[0] = CALIB25;
  i2c_write_blocking(i2c0, BME280_ADDR, data, 1, true);
  i2c_read_blocking(i2c0, BME280_ADDR, &calib.dig_H1, 1, false);

  data[0] = CALIB26;
  i2c_write_blocking(i2c0, BME280_ADDR, data, 1, true);
  i2c_read_blocking(i2c0, BME280_ADDR, data, 7, false);

  calib.dig_H2 = data[0] | data[1] << 8;
  calib.dig_H3 = data[2];
  calib.dig_H4 = data[3] << 4 | (data[4] & 0xF);
  calib.dig_H5 = data[5] << 4 | data[4] >> 4;
  calib.dig_H6 = data[6];

  blit::insert_api_sensor_data(&press_data);
  blit::insert_api_sensor_data(&temp_data);
  blit::insert_api_sensor_data(&hum_data);
}

static void update_bme280(uint32_t time) {
  if(!bme280_present)
    return;

  // probably shouldn't sample this too frequently...
  uint8_t raw_data[8];

  uint8_t reg = PRESS_MSB;
  i2c_write_blocking(i2c0, BME280_ADDR, &reg, 1, true);
  i2c_read_blocking(i2c0, BME280_ADDR, raw_data, 8, false);

  int32_t raw_pressure = raw_data[0] << 12 | raw_data[1] << 4 | raw_data[2];
  int32_t raw_temperature = raw_data[3] << 12 | raw_data[4] << 4 | raw_data[5];
  int32_t raw_humidity = raw_data[6] << 8 | raw_data[7];

  temp_data.data = bme280_compensate_T_int32(raw_temperature) / 100.0f;
  press_data.data = bme280_compensate_P_int64(raw_pressure) / 256.0f;
  hum_data.data = bme280_compensate_H_int32(raw_humidity) / 1024.0f;
}

SensorDriver bme280_driver {
  init_bme280, update_bme280
};
