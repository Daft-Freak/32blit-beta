#include "display.hpp"

#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

#include "gbacart.h"

#include "config.h"

// custom RGB555 blend code
inline uint32_t alpha(uint32_t a1, uint32_t a2) {
  return ((a1 + 1) * (a2 + 1)) >> 8;
}

inline uint32_t alpha(uint32_t a1, uint32_t a2, uint32_t a3) {
  return ((a1 + 1) * (a2 + 1) * (a3 + 1)) >> 16;
}

inline uint8_t blend(uint8_t s, uint8_t d, uint8_t a) {
  return d + ((a * (s - d) + 127) >> 8);
}

[[gnu::always_inline]] inline uint16_t pack_rgb555(uint8_t r, uint8_t g, uint8_t b) {
  return (r >> 3) | ((g >> 3) << 5) | ((b >> 3) << 10);
}

[[gnu::always_inline]] inline void unpack_rgb555(uint16_t rgb555, uint8_t &r, uint8_t &g, uint8_t &b) {
  r =  rgb555        & 0x1F; r = r << 3;
  g = (rgb555 >>  5) & 0x1F; g = g << 3;
  b = (rgb555 >> 10) & 0x1F; b = b << 3;
}

[[gnu::always_inline]] inline void blend_rgba_rgb555(const blit::Pen *s, uint8_t *d, uint8_t a, uint32_t c) {
  auto *d16 = (uint16_t *)d;
  uint8_t r, g, b;

  if (c == 1) {
    // fast case for single pixel draw
    unpack_rgb555(*d16, r, g, b);
    *d16 = pack_rgb555(blend(s->r, r, a), blend(s->g, g, a), blend(s->b, b, a));
    return;
  }

  // align
  auto de = d16 + c;
  if (uintptr_t(d) & 0b10) {
    unpack_rgb555(*d16, r, g, b);
    *d16++ = pack_rgb555(blend(s->r, r, a), blend(s->g, g, a), blend(s->b, b, a));
  }

  // destination is now aligned
  uint32_t *d32 = (uint32_t*)d16;

  // copy four bytes at a time until we have fewer than four bytes remaining
  uint32_t c32 = uint32_t(de - d16) >> 1;
  while (c32--) {
    uint8_t r2, g2, b2;

    unpack_rgb555(*d32, r, g, b);
    unpack_rgb555(*d32 >> 16, r2, g2, b2);

    *d32++ = pack_rgb555(blend(s->r, r, a), blend(s->g, g, a), blend(s->b, b, a))
            | pack_rgb555(blend(s->r, r2, a), blend(s->g, g2, a), blend(s->b, b2, a)) << 16;
  }

  // copy the trailing bytes as needed
  d16 = (uint16_t*)d32;
  if (d16 < de) {
    unpack_rgb555(*d16, r, g, b);
    *d16 = pack_rgb555(blend(s->r, r, a), blend(s->g, g, a), blend(s->b, b, a));
  }
}

[[gnu::always_inline]] inline void copy_rgba_rgb555(const blit::Pen* s, uint8_t *d, uint32_t c) {
  auto *d16 = (uint16_t *)d;
  uint16_t s16 = pack_rgb555(s->r, s->g, s->b);

  if (c == 1) {
    // fast case for single pixel draw
    *d16 = s16;
    return;
  }

  // align
  auto de = d16 + c;
  if (uintptr_t(d) & 0b10)
    *d16++ = s16;

  // destination is now aligned
  uint32_t *d32 = (uint32_t*)d16;

  // create packed 32bit source
  uint32_t s32 = s16 | (s16 << 16);

  // copy four bytes at a time until we have fewer than four bytes remaining
  uint32_t c32 = uint32_t(de - d16) >> 1;
  while (c32--)
    *d32++ = s32;

  // copy the trailing bytes as needed
  d16 = (uint16_t*)d32;
  if (d16 < de)
    *d16 = s16;
}

static void RGBA_RGB555(const blit::Pen* pen, const blit::Surface* dest, uint32_t off, uint32_t c) {
  if(!pen->a) return;

  uint8_t* d = dest->data + (off * 2);
  uint8_t* m = dest->mask ? dest->mask->data + off : nullptr;

  uint16_t a = alpha(pen->a, dest->alpha);
  if (!m) {
    // no mask
    if (a >= 255) {
      // no alpha, just copy
      copy_rgba_rgb555(pen, d, c);
    }
    else {
      // alpha, blend
      blend_rgba_rgb555(pen, d, a, c);
    }
  } else {
    // mask enabled, slow blend
    do {
      uint16_t ma = alpha(a, *m++);
      blend_rgba_rgb555(pen, d, ma, 1);
      d += 2;
    } while (--c);
  }
}

static void RGBA_RGB555(const blit::Surface* src, uint32_t soff, const blit::Surface* dest, uint32_t doff, uint32_t cnt, int32_t src_step) {
  uint8_t* s = src->palette ? src->data + soff : src->data + (soff * src->pixel_stride);
  uint8_t* d = dest->data + (doff * 2);
  uint8_t* m = dest->mask ? dest->mask->data + doff : nullptr;

  auto d16 = (uint16_t *)d;

  do {
    auto *pen = src->palette ? &src->palette[*s] : (blit::Pen *)s;

    uint16_t a = src->format == blit::PixelFormat::RGB ? 255 : pen->a;
    a = m ? alpha(a, *m++, dest->alpha) : alpha(a, dest->alpha);

    if (a >= 255) {
      *d16++ = pack_rgb555(pen->r, pen->g, pen->b);
    } else if (a > 1) {
      uint8_t r, g, b;
      unpack_rgb555(*d16, r, g, b);
      *d16++ = pack_rgb555(blend(pen->r, r, a), blend(pen->g, g, a), blend(pen->b, b, a));
    }else{
      d16++;
    }

    s += (src->pixel_stride) * src_step;
  } while (--cnt);
}

static blit::Pen get_pen_rgb555(const blit::Surface *surf, uint32_t offset) {
  auto ptr = surf->data + offset * 2;

  auto rgb555 = *(uint16_t *)ptr;
  uint8_t r, g, b;
  unpack_rgb555(rgb555, r, g, b);
  return {r, g, b};
}

void __no_inline_not_in_flash_func(core1_main)() {
    gbacart_init();

    auto cart_api = gbacart_get_api();
    cart_api->buttons = ~0;

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
  new_surf_template.pen_blend = RGBA_RGB555;
  new_surf_template.blit_blend = RGBA_RGB555;
  new_surf_template.pen_get = get_pen_rgb555;
}
