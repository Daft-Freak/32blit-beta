// battery voltage using ADC
#include "sensor.hpp"

#include "hardware/adc.h"

#include "config.h"

#include "engine/api_private.hpp"

static blit::SensorDataFloat batt_data(blit::SensorType::BATTERY_VOLTAGE);

static void init_battery_adc() {
  adc_init();
  adc_gpio_init(BATTERY_ADC_PIN);

  blit::insert_api_sensor_data(&batt_data);
}

static void update_battery_adc(uint32_t time) {
  adc_select_input(BATTERY_ADC_PIN - ADC_BASE_PIN);
  auto raw = adc_read();
  batt_data.data = (float(raw) * 3.3f * BATTERY_ADC_SCALE) / 4095.0f;
}

SensorDriver battery_adc_driver {
  init_battery_adc, update_battery_adc
};
