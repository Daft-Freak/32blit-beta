// LTR-559ALS-01 light + proximity
#include "sensor.hpp"

#include "hardware/i2c.h"

#include "math/constants.hpp"
#include "engine/api_private.hpp"

#ifndef LTR_559ALS_ADDR
#define LTR_559ALS_ADDR 0x23
#endif

static bool ltr_559_present = true;

static blit::SensorDataFloat light_data(blit::SensorType::LIGHT);
static blit::SensorDataFloat prox_data(blit::SensorType::PROXIMITY);

enum LTR_559ALSReg {
  ALS_CONTR = 0x80,
  PS_CONTR,
  PS_LED,
  PS_N_PULSES,
  PS_MEAS_RATE,
  ALS_MEAS_RATE,

  ALS_DATA_CH1 = 0x88,
  ALS_DATA_CH0 = 0x8A,

  ALS_PS_STATUS = 0x8C,

  PS_DATA = 0x8D
};

static void init_ltr_559() {
  // init
  uint8_t data[7];
  data[0] = ALS_CONTR;
  data[1] = 1; // active
  if(i2c_write_blocking(i2c0, LTR_559ALS_ADDR, data, 2, false) == PICO_ERROR_GENERIC) {
    ltr_559_present = false;
    return;
  }

  data[0] = PS_CONTR;
  data[1] = 3; // active
  i2c_write_blocking(i2c0, LTR_559ALS_ADDR, data, 2, false);

  // maybe configure some of the other regs?
  // datasheet doesn't really give any suggestions...

  blit::insert_api_sensor_data(&light_data);
  blit::insert_api_sensor_data(&prox_data);
}

static void update_ltr_559(uint32_t time) {
  if(!ltr_559_present)
    return;

  uint8_t status;

  uint8_t reg = ALS_PS_STATUS;
  i2c_write_blocking(i2c0, LTR_559ALS_ADDR, &reg, 1, true);
  i2c_read_blocking(i2c0, LTR_559ALS_ADDR, &status, 1, false);

  if(status & (1 << 0) /*proximity*/) {
    uint16_t raw_data;
    reg = PS_DATA;
    i2c_write_blocking(i2c0, LTR_559ALS_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, LTR_559ALS_ADDR, (uint8_t *)&raw_data, 2, false);

    // what does this value even mean?
    prox_data.data = raw_data & 0x7FF;
  }

  if(status & (1 << 2) /*light*/) {
    uint16_t raw_data[2]; // this is 1, 0
    reg = ALS_DATA_CH1;
    i2c_write_blocking(i2c0, LTR_559ALS_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, LTR_559ALS_ADDR, (uint8_t *)raw_data, 4, false);

    // now convert it
    auto ch1 = raw_data[0], ch0 = raw_data[1];
    float ratio = float(ch1) / (ch0 + ch1);

    const float gain = 1.0f;
    const float int_time = 100 / 100.0f;
    float lux = 0.0f;

    if(ratio < 0.45f)
      lux = (1.7743f * ch0 + 1.1059f * ch1) / gain / int_time;
    else if(ratio < 0.64f)
      lux = (4.2785f * ch0 - 1.9548f * ch1) / gain / int_time;
    else if(ratio < 0.85f)
      lux = (0.5926f * ch0 + 0.1185f * ch1) / gain / int_time;

    // TODO: compensation for window/aperture?

    light_data.data = lux;
  }
}

SensorDriver ltr_559als_driver {
  init_ltr_559, update_ltr_559
};
