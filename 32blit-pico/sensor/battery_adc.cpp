// battery voltage using ADC
#include "sensor.hpp"

#include "hardware/adc.h"
#include "pico/time.h"

#include "config.h"

#include "engine/api_private.hpp"

static blit::SensorDataFloat batt_data(blit::SensorType::BATTERY_VOLTAGE);

static void init_battery_adc() {
  adc_init();
  adc_gpio_init(BATTERY_ADC_PIN);

#ifdef BATTERY_ADC_REF_PIN
  adc_gpio_init(BATTERY_ADC_REF_PIN);
#endif

#ifdef BATTERY_ADC_REF_EN_PIN
  gpio_set_dir(BATTERY_ADC_REF_EN_PIN, GPIO_OUT);
  gpio_put(BATTERY_ADC_REF_EN_PIN, 0);
  gpio_set_function(BATTERY_ADC_REF_EN_PIN, GPIO_FUNC_SIO);
#endif

  blit::insert_api_sensor_data(&batt_data);
}

static void update_battery_adc(uint32_t time) {

  float vdd = 3.3f;

  // use reference to calculate true VDD
#ifdef BATTERY_ADC_REF_PIN

#ifdef BATTERY_ADC_REF_EN_PIN
  gpio_put(BATTERY_ADC_REF_EN_PIN, 1);
  sleep_us(10);
#endif

  adc_select_input(BATTERY_ADC_REF_PIN - ADC_BASE_PIN);

  vdd = 1.24f * (4095.0f / adc_read());

#ifdef BATTERY_ADC_REF_EN_PIN
  gpio_put(BATTERY_ADC_REF_EN_PIN, 0);
#endif
#endif

  // calculate battery voltage
  adc_select_input(BATTERY_ADC_PIN - ADC_BASE_PIN);
  auto raw = adc_read();
  batt_data.data = (float(raw) * vdd * BATTERY_ADC_SCALE) / 4095.0f;
}

extern const SensorDriver battery_adc_driver {
  init_battery_adc, update_battery_adc
};
