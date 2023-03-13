#include "engine/engine.hpp"
#include "engine/api_private.hpp"

#include "config.h"

#ifndef BLIT_BOARD_PIMORONI_PICOVISION

#if ALLOW_HIRES && DOUBLE_BUFFERED_HIRES
static const int fb_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * 2;
#elif ALLOW_HIRES
static const int fb_size = DISPLAY_WIDTH * DISPLAY_HEIGHT;
#else
// height rounded up to handle the 135px display
static const int fb_size = (DISPLAY_WIDTH / 2) * ((DISPLAY_HEIGHT + 1) / 2) * 2; // double-buffered
#endif

static uint16_t screen_fb[fb_size];
#endif

extern void init();
extern void update(uint32_t time);
extern void render(uint32_t time);

extern "C" bool do_init() {
#ifndef IGNORE_API_VERSION
  if(blit::api.version_major != blit::api_version_major)
    return false;

  if(blit::api.version_minor < blit::api_version_minor)
    return false;
#endif

  // preinit/init funcs (based on pico-sdk runtime.c)

  // Start and end points of the constructor list,
  // defined by the linker script.
  extern void (*__preinit_array_start)();
  extern void (*__preinit_array_end)();

  // Call each function in the list.
  // We have to take the address of the symbols, as __preinit_array_start *is*
  // the first function pointer, not the address of it.
  for (void (**p)(void) = &__preinit_array_start; p < &__preinit_array_end; ++p) {
    (*p)();
  }

  // Start and end points of the constructor list,
  // defined by the linker script.
  extern void (*__init_array_start)(void);
  extern void (*__init_array_end)(void);

  // Call each function in the list.
  // We have to take the address of the symbols, as __init_array_start *is*
  // the first function pointer, not the address of it.
  for (void (**p)(void) = &__init_array_start; p < &__init_array_end; ++p) {
    (*p)();
  }

  blit::update = update;
  blit::render = render;

#ifndef BLIT_BOARD_PIMORONI_PICOVISION
  blit::api.set_framebuffer((uint8_t *)screen_fb, sizeof(screen_fb), {DISPLAY_WIDTH, DISPLAY_HEIGHT});
#endif

  blit::set_screen_mode(blit::ScreenMode::lores);

  init();

  return true;
}

extern "C" void do_render(uint32_t time) {
  blit::screen.data = blit::api.get_screen_data();
  render(time);
}
