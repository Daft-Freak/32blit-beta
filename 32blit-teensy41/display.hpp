#pragma once

#include "engine/api_private.hpp"
#include "engine/engine.hpp"
#include "graphics/surface.hpp"

namespace display {
  void init();
  void update();

  bool set_screen_mode_format(blit::ScreenMode mode, blit::SurfaceTemplate &new_surf_template);
}
