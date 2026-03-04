#include <array>

#include "sensor.hpp"

#include "engine/api_private.hpp"

extern SensorDriver battery_adc_driver;
extern SensorDriver bme280_driver;
extern SensorDriver lsm6ds3tr_c_driver;
extern SensorDriver ltr_559als_driver;

static SensorDriver *sensor_drivers[] {
#ifdef BLIT_SENSOR_BATTERY_ADC
  &battery_adc_driver,
#endif
#ifdef BLIT_SENSOR_BME280
  &bme280_driver,
#endif
#ifdef BLIT_SENSOR_LSM6DS3TR_C
  &lsm6ds3tr_c_driver,
#endif
#ifdef BLIT_SENSOR_LTR_559ALS
  &ltr_559als_driver,
#endif
};
static constexpr unsigned num_sensor_drivers = sizeof(sensor_drivers) / sizeof(sensor_drivers[0]);

void init_sensors() {
  blit::api_data.sensors = nullptr;

  for(unsigned i = 0; i < num_sensor_drivers; i++)
    sensor_drivers[i]->init();
}

void update_sensors(uint32_t time) {
  for(unsigned i = 0; i < num_sensor_drivers; i++)
    sensor_drivers[i]->update(time);
}
