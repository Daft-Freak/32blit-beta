#include <cstring>

#include "driver/gpio.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "rom/cache.h"

#include "display.hpp"

#include "config.h"

// mode (default to 640x480)
#ifndef DPI_MODE_CLOCK
#define DPI_MODE_CLOCK 25000000
#endif

#ifndef DPI_MODE_H_SYNC_POLARITY
#define DPI_MODE_H_SYNC_POLARITY 0
#endif
#ifndef DPI_MODE_H_FRONT_PORCH
#define DPI_MODE_H_FRONT_PORCH   16
#endif
#ifndef DPI_MODE_H_SYNC_WIDTH
#define DPI_MODE_H_SYNC_WIDTH    96
#endif
#ifndef DPI_MODE_H_BACK_PORCH
#define DPI_MODE_H_BACK_PORCH    48
#endif
#ifndef DPI_MODE_H_ACTIVE_PIXELS
#define DPI_MODE_H_ACTIVE_PIXELS 640
#endif

#ifndef DPI_MODE_V_SYNC_POLARITY
#define DPI_MODE_V_SYNC_POLARITY 0
#endif
#ifndef DPI_MODE_V_FRONT_PORCH
#define DPI_MODE_V_FRONT_PORCH   10
#endif
#ifndef DPI_MODE_V_SYNC_WIDTH
#define DPI_MODE_V_SYNC_WIDTH    2
#endif
#ifndef DPI_MODE_V_BACK_PORCH
#define DPI_MODE_V_BACK_PORCH    33
#endif
#ifndef DPI_MODE_V_ACTIVE_LINES
#define DPI_MODE_V_ACTIVE_LINES  480
#endif

static_assert(DPI_MODE_H_ACTIVE_PIXELS % DISPLAY_WIDTH == 0);
static_assert(DPI_MODE_V_ACTIVE_LINES % DISPLAY_HEIGHT == 0);

static esp_lcd_panel_handle_t panel_handle = nullptr;

static uint16_t *display_buffers[2];
static int buf_index = 0;
static bool display_update_done = true;

#ifdef LCD_BACKLIGHT_PIN
static bool backlight_enabled = false;
#endif

static void *alloc_display_buffer() {
  return new uint16_t[DISPLAY_WIDTH * DISPLAY_HEIGHT];
}

static bool on_bounce_empty_1x(esp_lcd_panel_handle_t panel, void *bounce_buf, int pos_px, int len_bytes, void *user_ctx) {

  auto in = display_buffers[buf_index ^ 1] + pos_px;

  memcpy(bounce_buf, in, len_bytes);

  // preload next (this is mimicking what the default code does)
#if CONFIG_IDF_TARGET_ESP32P4
    Cache_Start_L2_Cache_Preload(uint32_t(in + len_bytes / sizeof(uint16_t)), len_bytes, 0);
#endif

  return false;
}

static bool on_bounce_empty_2x(esp_lcd_panel_handle_t panel, void *bounce_buf, int pos_px, int len_bytes, void *user_ctx) {
  auto buf32 = reinterpret_cast<uint32_t *>(bounce_buf);

  // buffer is a multiple of line size so we should have an even number
  int pos_y = pos_px / DPI_MODE_H_ACTIVE_PIXELS;
  int out_lines = len_bytes / (DPI_MODE_H_ACTIVE_PIXELS * sizeof(uint16_t));

  int in_y = pos_y / 2;
  auto in = display_buffers[buf_index ^ 1] + in_y * (DPI_MODE_H_ACTIVE_PIXELS / 2);

  for(int y = 0; y < out_lines; y += 2) {
    // pixel-double line
    auto line_in = in;
    for(int x = 0; x < DPI_MODE_H_ACTIVE_PIXELS / 2; x++) {
      uint16_t px = *line_in++;
      *buf32++ = px | px << 16;
    }

    // repeat line
    for(int x = 0; x < DPI_MODE_H_ACTIVE_PIXELS / 2; x++) {
      uint16_t px = *in++;
      *buf32++ = px | px << 16;
    }
  }

  return false;
}

static bool on_bounce_empty_4x(esp_lcd_panel_handle_t panel, void *bounce_buf, int pos_px, int len_bytes, void *user_ctx) {
  auto buf32 = reinterpret_cast<uint32_t *>(bounce_buf);

  // buffer is a multiple of line size so we should have an even number
  int pos_y = pos_px / DPI_MODE_H_ACTIVE_PIXELS;
  int out_lines = len_bytes / (DPI_MODE_H_ACTIVE_PIXELS * sizeof(uint16_t));

  int in_y = pos_y / 4;
  auto in = display_buffers[buf_index ^ 1] + in_y * (DPI_MODE_H_ACTIVE_PIXELS / 4);

  for(int y = 0; y < out_lines; y += 4) {
    // repeat pixels four times
    // and repeat line three times
    for(int i = 0; i < 3; i++) {
      auto line_in = in;
      for(int x = 0; x < DPI_MODE_H_ACTIVE_PIXELS / 4; x++) {
        uint16_t px = *line_in++;
        *buf32++ = px | px << 16;
        *buf32++ = px | px << 16;
      }
    }

    // repeat final line
    for(int x = 0; x < DPI_MODE_H_ACTIVE_PIXELS / 4; x++) {
      uint16_t px = *in++;
      *buf32++ = px | px << 16;
      *buf32++ = px | px << 16;
    }
  }

  return false;
}

static bool on_frame_buf_complete(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx) {
#ifdef LCD_BACKLIGHT_PIN
  // enable backlight
  if(!backlight_enabled) {
    gpio_set_level(gpio_num_t(LCD_BACKLIGHT_PIN), 1);
    backlight_enabled = true;
  }
#endif

  if(!display_update_done) {
    display_update_done = true;

    // swap used buffer
    buf_index ^= 1;
    blit::screen.data = (uint8_t *)display_buffers[buf_index];
  }

  return false;
}

static void init_callbacks(int scale) {
  auto on_bounce_empty = on_bounce_empty_1x;

  if(scale == 2)
    on_bounce_empty = on_bounce_empty_2x;
  else if(scale == 4)
    on_bounce_empty = on_bounce_empty_4x;

  esp_lcd_rgb_panel_event_callbacks_t callbacks = {
    .on_color_trans_done = nullptr,
    .on_vsync = nullptr,
    .on_bounce_empty = on_bounce_empty,
    .on_frame_buf_complete = on_frame_buf_complete,
  };
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &callbacks, nullptr));
}

void init_display() {

#ifdef LCD_BACKLIGHT_PIN
  // backlight
  gpio_config_t backlight_gpio_config = {};
  backlight_gpio_config.mode = GPIO_MODE_OUTPUT;
  backlight_gpio_config.pin_bit_mask = 1ULL << LCD_BACKLIGHT_PIN;

  ESP_ERROR_CHECK(gpio_config(&backlight_gpio_config));
  gpio_set_level(gpio_num_t(LCD_BACKLIGHT_PIN), 0);
#endif

  // rgb panel init
  esp_lcd_rgb_panel_config_t panel_config = {
    .clk_src = LCD_CLK_SRC_DEFAULT,
    .timings = {
      .pclk_hz = DPI_MODE_CLOCK,
      .h_res = DPI_MODE_H_ACTIVE_PIXELS,
      .v_res = DPI_MODE_V_ACTIVE_LINES,
      .hsync_pulse_width = DPI_MODE_H_SYNC_WIDTH,
      .hsync_back_porch = DPI_MODE_H_BACK_PORCH,
      .hsync_front_porch = DPI_MODE_H_FRONT_PORCH,
      .vsync_pulse_width = DPI_MODE_V_SYNC_WIDTH,
      .vsync_back_porch = DPI_MODE_V_BACK_PORCH,
      .vsync_front_porch = DPI_MODE_V_FRONT_PORCH,
      .flags = {}
    },
    .data_width = 16,
    .bits_per_pixel = 0,
    .num_fbs = 0,
    .bounce_buffer_size_px = 20 * DPI_MODE_H_ACTIVE_PIXELS,
    .sram_trans_align = 0,
    .dma_burst_size = 64,
    .hsync_gpio_num = LCD_HSYNC_PIN,
    .vsync_gpio_num = LCD_VSYNC_PIN,
    .de_gpio_num = LCD_DE_PIN,
    .pclk_gpio_num = LCD_CLOCK_PIN,
    .disp_gpio_num = -1,
    .data_gpio_nums = {
      LCD_DATA0_PIN,
      LCD_DATA1_PIN,
      LCD_DATA2_PIN,
      LCD_DATA3_PIN,
      LCD_DATA4_PIN,
      LCD_DATA5_PIN,
      LCD_DATA6_PIN,
      LCD_DATA7_PIN,
      LCD_DATA8_PIN,
      LCD_DATA9_PIN,
      LCD_DATA10_PIN,
      LCD_DATA11_PIN,
      LCD_DATA12_PIN,
      LCD_DATA13_PIN,
      LCD_DATA14_PIN,
      LCD_DATA15_PIN,
    },

    .flags = {
      .disp_active_low = false,
      .refresh_on_demand = false,
      .fb_in_psram = false,
      .double_fb = false,
      .no_fb = true,
      .bb_invalidate_cache = false,
    },
  };
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  init_callbacks(DPI_MODE_H_ACTIVE_PIXELS / DISPLAY_WIDTH);

  // alloc buffers
  display_buffers[0] = (uint16_t *)alloc_display_buffer();
  display_buffers[1] = (uint16_t *)alloc_display_buffer();

  if(!display_buffers[1])
    display_buffers[1] = display_buffers[0];
}

void update_display(uint32_t time) {
  // can't do anything if last update still in progress
  if(!display_update_done)
    return;

  blit::render(time);

  display_update_done = false;
}

bool display_render_needed() {
  return false;
}

bool display_mode_supported(blit::ScreenMode new_mode, const blit::SurfaceTemplate &new_surf_template) {
  if(new_surf_template.format != blit::PixelFormat::RGB565)
    return false;

  // lores
  if(new_surf_template.bounds == blit::Size{DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2})
    return true;

  return new_surf_template.bounds == blit::Size{DISPLAY_WIDTH, DISPLAY_HEIGHT};
}

void display_mode_changed(blit::ScreenMode new_mode, blit::SurfaceTemplate &new_surf_template) {
  new_surf_template.data = blit::screen.data = (uint8_t *)display_buffers[buf_index];

  // hmm, should delay callback switch until end of frame
  init_callbacks(DPI_MODE_H_ACTIVE_PIXELS / new_surf_template.bounds.w);
}
