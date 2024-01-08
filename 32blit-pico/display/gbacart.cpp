#include "display.hpp"

#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

#include "gbacart.h"

#include "config.h"

void __no_inline_not_in_flash_func(core1_main)() {
    gbacart_init();
    gbacart_start(true);

    while(true) {
      __wfe();
    }
}

void init_display() {
  // take over core1
  multicore_launch_core1(core1_main);

  // TODO: setting lores later will fail
  blit::set_screen_mode(blit::ScreenMode::hires);
}

void update_display(uint32_t time) {
  auto cart_api = gbacart_get_api();

  if(cart_api->vblank_flag) {
    blit::render(time);

    cart_api->vblank_flag = 0;
    cart_api->fb_addr = gbacart_to_gba_addr(screen_fb);
  }
}

void init_display_core1() {
}

void update_display_core1() {
}

bool display_render_needed() {
  return false;
}

bool display_mode_supported(blit::ScreenMode new_mode, const blit::SurfaceTemplate &new_surf_template) {
  // FIXME: 555
  if(new_surf_template.format != blit::PixelFormat::RGB565)
    return false;

  // TODO: can scale
  blit::Size expected_bounds(DISPLAY_WIDTH, DISPLAY_HEIGHT);

  if(new_surf_template.bounds == expected_bounds)
    return true;

  return false;
}

void display_mode_changed(blit::ScreenMode new_mode, blit::SurfaceTemplate &new_surf_template) {
}
