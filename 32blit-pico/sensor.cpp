#include <array>

#include "sensor.hpp"

#include "engine/api_private.hpp"

static const SensorDriver *sensor_drivers[] {
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
