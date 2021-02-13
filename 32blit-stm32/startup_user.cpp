#include "engine/engine.hpp"
#include "engine/api_private.hpp"

extern void init();
extern void update(uint32_t time);
extern void render(uint32_t time);

extern "C" bool cpp_do_init() {
    if(blit::api.version < blit::api_version)
        return false;

    blit::update = update;
    blit::render = render;

    blit::set_screen_mode(blit::ScreenMode::lores);

    init();

    return true;
}

extern "C" void _exit(int code) {
  blit::api.exit(code != 0);
}

extern "C" void *_sbrk(ptrdiff_t incr) {
  extern char end, __ltdc_start;
  static char *cur_heap_end;

  // ltdc is at the end of the heap
  auto heap_end = &__ltdc_start;

  // paletted is shifted forward 75k
  if(blit::screen.format == blit::PixelFormat::P)
    heap_end += 320 * 240;

  if(!cur_heap_end)
    cur_heap_end = &end;

  // check for room
  if(cur_heap_end + incr > heap_end) {
    return (void *)-1;
  }

  char *ret = cur_heap_end;
  cur_heap_end += incr;

  return (void *)ret;
}
