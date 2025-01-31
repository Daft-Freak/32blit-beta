/*! \file input.cpp
    \brief Input handlers
*/
#include "input.hpp"
#include "api.hpp"
#include "api_private.hpp"

namespace blit {

  /**
   * Return pressed state of a button or buttons.
   *
   * \param button Bitmask for button(s) to read.
   * \return `true` for pressed, `false` for released.
   */
  bool pressed(uint32_t button) {
    return buttons.state & button;
  }

  Sensor::Sensor() : data (nullptr){
  }

  Sensor::Sensor(SensorType type) : data(nullptr) {
    for(auto sensor = api_data.sensors; sensor; sensor = sensor->next) {
      if(sensor->type == type)
        data = sensor;
    }
  }

  bool Sensor::is_present() const {
    return data != nullptr;
  }

  float Sensor::get_float() const {
    if(!is_present())
      return {};

    auto float_data = (SensorDataFloat *)data;

    if(float_data->data_type != SensorDataType::FLOAT)
      return {};

    return float_data->data;
  }

  Vec3 Sensor::get_vec3() const {
    if(!is_present())
      return {};

    auto vec3_data = (SensorDataVec3 *)data;

    if(vec3_data->data_type != SensorDataType::VEC3)
      return {};

    return vec3_data->data;
  }
}
