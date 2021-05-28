/*! \file input.cpp
    \brief Input handlers
*/
#include "input.hpp"
#include "api.hpp"

namespace blit {

  /**
   * Maps the joystick to the DPAD when above this value, set to a value < 1 to enable
   */
  float joystick_dpad_threshold = 1.0f;

  /**
   * Return pressed state of a button or buttons.
   *
   * \param button Bitmask for button(s) to read.
   * \return `true` for pressed, `false` for released.
   */
  bool pressed(uint32_t button) {
    return buttons.state & button;
  }

}
