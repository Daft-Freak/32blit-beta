#pragma once

#include "engine/engine.hpp"
#include "graphics/surface.hpp"

namespace display {
  void init();
  void update();

  blit::Surface &set_screen_mode(blit::ScreenMode mode);
}