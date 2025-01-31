#include <array>

#include "sensor.hpp"

extern SensorDriver lsm6ds3tr_c_driver;

static SensorDriver *sensor_drivers[] {
#ifdef BLIT_SENSOR_LSM6DS3TR_C
  &lsm6ds3tr_c_driver
#endif
};
static constexpr unsigned num_sensor_drivers = sizeof(sensor_drivers) / sizeof(sensor_drivers[0]);

void init_sensors() {
  for(unsigned i = 0; i < num_sensor_drivers; i++)
    sensor_drivers[i]->init();
}

void update_sensors(uint32_t time) {
  for(unsigned i = 0; i < num_sensor_drivers; i++)
    sensor_drivers[i]->update(time);
}
