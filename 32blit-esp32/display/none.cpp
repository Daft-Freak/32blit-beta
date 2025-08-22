#include "display.hpp"

#include "config.h"

static uint32_t last_render = 0;

void init_display() {
}

void update_display(uint32_t time) {
  // render timing placeholder
  if(time - last_render >= 20) {
    blit::render(time);
    last_render = time;
  }
}

bool display_render_needed() {
  return false;
}

bool display_mode_supported(blit::ScreenMode new_mode, const blit::SurfaceTemplate &new_surf_template) {
  return false;
}

void display_mode_changed(blit::ScreenMode new_mode, blit::SurfaceTemplate &new_surf_template) {
}
