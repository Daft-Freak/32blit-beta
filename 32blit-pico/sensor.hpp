#pragma once
#include <cstdint>

struct SensorDriver {
  void (*init)();
  void (*update)(uint32_t);
};

void init_sensors();
void update_sensors(uint32_t time);
