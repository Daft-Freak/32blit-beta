#include "driver/gpio.h"
#include "driver/ppa.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "display.hpp"

#include "config.h"

static uint32_t last_render = 0;
static esp_lcd_panel_io_handle_t io_handle = nullptr;
static esp_lcd_panel_handle_t panel_handle = nullptr;

#if SOC_PPA_SUPPORTED
static ppa_client_handle_t ppa_client = nullptr;
static int ppa_trans_steps = 0;
#endif

static uint16_t *display_buffers[2];
static int buf_index = 0;
static bool display_update_done = true;

static bool backlight_enabled = false;

static void *alloc_display_buffer() {
  // this depends on the bus...
#ifdef LCD_I80
  return esp_lcd_i80_alloc_draw_buffer(io_handle, DISPLAY_WIDTH * DISPLAY_HEIGHT * 2, 0);
#else
  return nullptr;
#endif
}

static bool on_color_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {

  // enable backlight
  if(!backlight_enabled) {
    gpio_set_level(gpio_num_t(LCD_BACKLIGHT_PIN), 1);
    backlight_enabled = true;
  }

  display_update_done = true;

  return false;
}

#if SOC_PPA_SUPPORTED
static bool on_ppa_trans_done(ppa_client_handle_t ppa_client, ppa_event_data_t *event_data, void *user_data) {
  // if we have two buffers, we're done using the drawn to one after the first copy
  if(display_buffers[1] != display_buffers[0])
    display_update_done = true;

  if(--ppa_trans_steps > 0)
    return false;

  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, display_buffers[1]);
  return false;
}
#endif

void init_display() {
  // backlight
  gpio_config_t backlight_gpio_config = {};
  backlight_gpio_config.mode = GPIO_MODE_OUTPUT;
  backlight_gpio_config.pin_bit_mask = 1ULL << LCD_BACKLIGHT_PIN;

  ESP_ERROR_CHECK(gpio_config(&backlight_gpio_config));
  gpio_set_level(gpio_num_t(LCD_BACKLIGHT_PIN), 0);

#ifdef LCD_I80
  // init "I80" bus (8-bit)
  esp_lcd_i80_bus_handle_t i80_bus = nullptr;

  esp_lcd_i80_bus_config_t bus_config = {};

  bus_config.clk_src = LCD_CLK_SRC_DEFAULT;
  bus_config.dc_gpio_num = LCD_DC_PIN;
  bus_config.wr_gpio_num = LCD_WR_PIN;
  bus_config.data_gpio_nums[0] = LCD_DATA0_PIN;
  bus_config.data_gpio_nums[1] = LCD_DATA1_PIN;
  bus_config.data_gpio_nums[2] = LCD_DATA2_PIN;
  bus_config.data_gpio_nums[3] = LCD_DATA3_PIN;
  bus_config.data_gpio_nums[4] = LCD_DATA4_PIN;
  bus_config.data_gpio_nums[5] = LCD_DATA5_PIN;
  bus_config.data_gpio_nums[6] = LCD_DATA6_PIN;
  bus_config.data_gpio_nums[7] = LCD_DATA7_PIN;

  bus_config.bus_width = 8;
  bus_config.max_transfer_bytes = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
  bus_config.dma_burst_size = 64;

  ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

  // init panel io
  esp_lcd_panel_io_i80_config_t io_config = {};
  io_config.cs_gpio_num = LCD_CS_PIN,
  io_config.pclk_hz = LCD_CLOCK,
  io_config.trans_queue_depth = 10,

  io_config.dc_levels.dc_idle_level = 0,
  io_config.dc_levels.dc_cmd_level = 0,
  io_config.dc_levels.dc_dummy_level = 0,
  io_config.dc_levels.dc_data_level = 1,

  io_config.lcd_cmd_bits = 8,
  io_config.lcd_param_bits = 8,
  io_config.flags.swap_color_bytes = 1;

  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));
#endif

  esp_lcd_panel_io_callbacks_t io_callbacks = {};
  io_callbacks.on_color_trans_done = on_color_trans_done;

  ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &io_callbacks, nullptr));

  // init panel
  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = LCD_RESET_PIN;
  panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
  panel_config.bits_per_pixel = 16;

#ifdef LCD_ILI9488
  // using the st7789 driver, but with a different init sequence
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);

#ifdef LCD_ILI9488_IPS
  esp_lcd_panel_invert_color(panel_handle, true);
#else
  esp_lcd_panel_invert_color(panel_handle, false);
#endif

  esp_lcd_panel_set_gap(panel_handle, 0, 0);
  esp_lcd_panel_swap_xy(panel_handle, true);
  esp_lcd_panel_mirror(panel_handle, true, true);

  if(DISPLAY_WIDTH == 480 && DISPLAY_HEIGHT == 320)
  {
    // TODO: I don't even know if this is correct for the display I have.
    esp_lcd_panel_io_tx_param(io_handle, 0xC0/*PWCTRL1*/, (uint8_t[]) {
      0x17, 0x12
    }, 2);

    esp_lcd_panel_io_tx_param(io_handle, 0xC1/*PWCTRL2*/, (uint8_t[]) {
      0x41
    }, 1);

#ifdef LCD_ILI9488_IPS
    esp_lcd_panel_io_tx_param(io_handle, 0xC5/*VMCTRL*/, (uint8_t[]) {
        0x00, 0x4D, 0x80
    }, 3);
#else
    esp_lcd_panel_io_tx_param(io_handle, 0xC5/*VMCTRL*/, (uint8_t[]) {
        0x00, 0x12, 0x80
    }, 3);
#endif

    esp_lcd_panel_io_tx_param(io_handle, 0xB1/*FRMCTR1*/, (uint8_t[]) {
      0xA0
    }, 1);

    esp_lcd_panel_io_tx_param(io_handle, 0xB7/*ETMOD*/, (uint8_t[]) {
      0x86
    }, 1);

    esp_lcd_panel_io_tx_param(io_handle, 0xE0/*PGAMCTRL*/, (uint8_t[]) {
      0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F
    }, 15);

    esp_lcd_panel_io_tx_param(io_handle, 0xE1/*NGAMCTRL*/, (uint8_t[]) {
      0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F
    }, 15);
  }
#elif defined(LCD_ST7789)
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);

  esp_lcd_panel_invert_color(panel_handle, false);
  esp_lcd_panel_set_gap(panel_handle, 0, 0);
  //esp_lcd_panel_swap_xy(panel_handle, true);
  //esp_lcd_panel_mirror(panel_handle, false, true);

  if(DISPLAY_WIDTH == 320 && DISPLAY_HEIGHT == 240) {
    esp_lcd_panel_io_tx_param(io_handle, 0xB2/*PORCTRL*/, (uint8_t[]) {
      0x0c, 0x0c, 0x00, 0x33, 0x33
    }, 5);

    esp_lcd_panel_io_tx_param(io_handle, 0xB7/*GCTRL*/, (uint8_t[]) {
      0x35
    }, 1);

    esp_lcd_panel_io_tx_param(io_handle, 0xBB/*VCOMS*/, (uint8_t[]) {
      0x1f
    }, 1);

    esp_lcd_panel_io_tx_param(io_handle, 0xC0/*LCMCTRL*/, (uint8_t[]) {
      0x1f
    }, 1);

    esp_lcd_panel_io_tx_param(io_handle, 0xC2/*VDVVRHEN*/, (uint8_t[]) {
      0x01
    }, 1);

    esp_lcd_panel_io_tx_param(io_handle, 0xC3/*VRHS*/, (uint8_t[]) {
      0x12
    }, 1);

    esp_lcd_panel_io_tx_param(io_handle, 0xC4/*VDVS*/, (uint8_t[]) {
      0x20
    }, 1);

    esp_lcd_panel_io_tx_param(io_handle, 0xD0/*PWCTRL1*/, (uint8_t[]) {
      0xa4, 0xa1
    }, 2);

    esp_lcd_panel_io_tx_param(io_handle, 0xD6/*???*/, (uint8_t[]) {
      0xa1
    }, 1);

    esp_lcd_panel_io_tx_param(io_handle, 0xE0/*PVGAMCTRL*/, (uint8_t[]) {
      0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2D
    }, 14);

    esp_lcd_panel_io_tx_param(io_handle, 0xE1/*NVGAMCTRL*/, (uint8_t[]) {
      0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0B, 0x16, 0x14, 0x2F, 0x31
    }, 14);
  }
#endif

  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

  // alloc buffers
  display_buffers[0] = (uint16_t *)alloc_display_buffer();
  display_buffers[1] = (uint16_t *)alloc_display_buffer();

  if(!display_buffers[1])
    display_buffers[1] = display_buffers[0];

  // pixel-processing accelerator
#if SOC_PPA_SUPPORTED
  ppa_client_config_t ppa_config = {};
  ppa_config.oper_type = PPA_OPERATION_SRM;
  ppa_config.max_pending_trans_num = 5;
  ppa_config.data_burst_length = PPA_DATA_BURST_LENGTH_128;
  ESP_ERROR_CHECK(ppa_register_client(&ppa_config, &ppa_client));

  ppa_event_callbacks_t ppa_callbacks = {};
  ppa_callbacks.on_trans_done = on_ppa_trans_done;
  ESP_ERROR_CHECK(ppa_client_register_event_callbacks(ppa_client, &ppa_callbacks));
#endif
}

void update_display(uint32_t time) {
  // can't do anything if last update still in progress
  if(!display_update_done)
    return;

  // render timing placeholder
  if(time - last_render >= 20) {
    blit::render(time);

#if SOC_PPA_SUPPORTED
    display_update_done = false;

    bool hires = blit::screen.bounds.w == DISPLAY_WIDTH;
    bool single_buf = display_buffers[1] == display_buffers[0];

    // if we have PPA, do a copy to the screen buffer so we can scale
    // this is also closer to the original 32blit behaviour
    ppa_srm_oper_config_t copy_config = {};

    copy_config.in.buffer = blit::screen.data;
    copy_config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;

    copy_config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;

    copy_config.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
    copy_config.scale_x = 1;
    copy_config.scale_y = 1;
    copy_config.mode = PPA_TRANS_MODE_NON_BLOCKING;

    if(hires && single_buf) {
      // if we only have one buffer, draw it directly
      esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, display_buffers[1]);
    } else if(hires) {
      ppa_trans_steps = 1; // only one transfer
      // 1:1 copy
      copy_config.in.buffer = blit::screen.data;
      copy_config.in.pic_w = copy_config.in.block_w = blit::screen.bounds.w;
      copy_config.in.pic_h = copy_config.in.block_h = blit::screen.bounds.h;

      copy_config.out.buffer = display_buffers[1];
      copy_config.out.buffer_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * 2;
      copy_config.out.pic_w = DISPLAY_WIDTH;
      copy_config.out.pic_h = DISPLAY_HEIGHT;

      ppa_do_scale_rotate_mirror(ppa_client, &copy_config);
    } else {
      // reinterpret as 1px wide
      // and copy to a 2px wide image
      // max size for DMA2D seems to be 8K, so we have to break this up

      int steps = std::ceil(blit::screen.bounds.area() / 8192.0f);
      ppa_trans_steps = steps + 2; // +2 for vertical copies

      int h = blit::screen.bounds.w * (blit::screen.bounds.h / steps);

      // copy into the bottom half of the dest
      auto temp_buf = display_buffers[1] + DISPLAY_WIDTH * (DISPLAY_HEIGHT / 2);

      auto in_ptr = reinterpret_cast<uint16_t *>(blit::screen.data);
      auto out_ptr = temp_buf;

      copy_config.in.pic_w = copy_config.in.block_w = 1;
      copy_config.in.pic_h = copy_config.in.block_h = h;

      copy_config.out.buffer_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * 2;
      copy_config.out.pic_w = 2;
      copy_config.out.pic_h = h;

      copy_config.scale_x = 2;

      for(int i = 0; i < steps; i++) {
        copy_config.in.buffer = in_ptr;
        copy_config.out.buffer = out_ptr;

        ppa_do_scale_rotate_mirror(ppa_client, &copy_config);

        // next
        in_ptr += h;
        out_ptr += h * 2;
      }

      // now we've done the horizontal double, do a couple more copies for the vertical double
      // copy from width * half height to double width * half height
      // (effectively skipping a line after each line)

      copy_config.in.buffer = temp_buf;
      copy_config.in.pic_w = copy_config.in.block_w = DISPLAY_WIDTH;
      copy_config.in.pic_h = copy_config.in.block_h = blit::screen.bounds.h;

      copy_config.out.buffer = display_buffers[1];
      copy_config.out.pic_w = DISPLAY_WIDTH * 2;
      copy_config.out.pic_h = blit::screen.bounds.h;

      copy_config.scale_x = 1;

      ppa_do_scale_rotate_mirror(ppa_client, &copy_config);

      // then copy it again to the other half
      copy_config.in.buffer = display_buffers[1];
      copy_config.in.pic_w *= 2;
      copy_config.out.buffer = display_buffers[1] + DISPLAY_WIDTH;
      ppa_do_scale_rotate_mirror(ppa_client, &copy_config);
    }

#else
    // send to display and swap buffers
    buf_index ^= 1;
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, blit::screen.data);
    blit::screen.data = (uint8_t *)display_buffers[buf_index];
#endif

    last_render = time;
  }
}

bool display_render_needed() {
  return false;
}

bool display_mode_supported(blit::ScreenMode new_mode, const blit::SurfaceTemplate &new_surf_template) {
  if(new_surf_template.format != blit::PixelFormat::RGB565)
    return false;

#if SOC_PPA_SUPPORTED
  // lores
  if(new_surf_template.bounds == blit::Size{DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2})
    return true;
#endif

  return new_surf_template.bounds == blit::Size{DISPLAY_WIDTH, DISPLAY_HEIGHT};
}

void display_mode_changed(blit::ScreenMode new_mode, blit::SurfaceTemplate &new_surf_template) {
  new_surf_template.data = blit::screen.data = (uint8_t *)display_buffers[buf_index];
}
