#pragma once

#include "../types/vec2.hpp"
#include "../types/vec3.hpp"
#include <cstdint>

namespace blit {
  enum Button : unsigned int {
    DPAD_LEFT = 1,
    DPAD_RIGHT = 2,
    DPAD_UP = 4,
    DPAD_DOWN = 8,
    A = 16,
    B = 32,
    X = 64,
    Y = 128,
    MENU = 256,
    HOME = 512,
    JOYSTICK = 1024
  };

  struct ButtonState {
    ButtonState &operator=(uint32_t v) {
      state = v;

      return *this;
    }

    operator uint32_t() const {
      return state;
    }

    uint32_t state;
    uint32_t pressed, released; // state change since last update
  };

  extern bool pressed(uint32_t button);

  enum class SensorType : uint8_t {
    ACCELEROMETER,
    GYROSCOPE,
    TEMPERATURE,
    PRESSURE,
    HUMIDITY,
    LIGHT,
    PROXIMITY,
  };

  enum class SensorDataType : uint8_t {
    FLOAT,
    VEC3,
  };

  struct Sensor {
    Sensor();
    Sensor(SensorType type);

    bool is_present() const;

    float get_float() const;
    Vec3 get_vec3() const;

  private:
    void *data;
  };
}
