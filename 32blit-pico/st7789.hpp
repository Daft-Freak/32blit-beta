#pragma once

#include "hardware/pio.h"
#include "hardware/gpio.h"

namespace st7789 {

  extern uint16_t *frame_buffer, *palette;

  void init(bool auto_init_sequence = true);

  void command(uint8_t command, size_t len = 0, const char *data = NULL);
  bool vsync_callback(gpio_irq_callback_t callback);
  void update(bool dont_block = false);
  void set_backlight(uint8_t brightness);

  void set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  void set_pixel_double(bool pd);
  void set_palette_mode(bool pal);

  void clear();

  bool dma_is_busy();
}
