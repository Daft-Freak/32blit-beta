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

  Sensor::Sensor(SensorType type) : data(nullptr) {
    for(auto sensor = api_data.sensors; sensor; sensor = sensor->next) {
      if(sensor->type == type)
        data = sensor;
    }
  }

  bool Sensor::is_present() const {
    return data != nullptr;
  }

  Vec3 Sensor::get_vec3() const {
    if(!is_present())
      return {};

    // check expected data type?

    return ((SensorDataVec3 *)data)->data;
  }
}
