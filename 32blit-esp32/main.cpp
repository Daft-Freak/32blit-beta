#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gptimer.h"

#include "engine/api_private.hpp"

#include "display.hpp"

static const blit::Size lores_screen_size(DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2);
static const blit::Size hires_screen_size(DISPLAY_WIDTH, DISPLAY_HEIGHT);

static uint32_t now();
static void debug(const char *str);
static bool set_screen_mode_format(blit::ScreenMode new_mode, blit::SurfaceTemplate &new_surf_template);

// blit API
static const blit::APIConst blit_api_const {
  blit::api_version_major, blit::api_version_minor,

  nullptr, // channels

  nullptr, // set_screen_mode
  nullptr, // set_screen_palette

  ::now,
  nullptr, // random
  nullptr, // exit
  ::debug,

  nullptr, // open_file
  nullptr, // read_file
  nullptr, // write_file
  nullptr, // close_file
  nullptr, // get_file_length
  nullptr, // list_files
  nullptr, // file_exists
  nullptr, // directory_exists
  nullptr, // create_directory
  nullptr, // rename_file
  nullptr, // remove_file
  nullptr, // get_save_path
  nullptr, // is_storage_available

  nullptr, // enable_us_timer
  nullptr, // get_us_timer
  nullptr, // get_max_us_timer

  nullptr, // decode_jpeg_buffer
  nullptr, // decode_jpeg_file

  nullptr, // launch_file
  nullptr, // erase_game
  nullptr, // get_type_handler_metadata

  nullptr, // get_launch_path

  nullptr, // is_multiplayer_connected
  nullptr, // set_multiplayer_enabled
  nullptr, // send_multiplayer_message

  nullptr, // flash_to_tmp
  nullptr, // tmp_file_closed

  nullptr, // get_metadata

  ::set_screen_mode_format,

  nullptr, // i2c_send
  nullptr, // i2c_recieve

  nullptr, // set_raw_cdc_enabled
  nullptr, // cdc_write
  nullptr, // cdc_read

  nullptr, // list_installed_games
  nullptr, // can_launch

  nullptr, // get_screen_data
  nullptr, // set_framebuffer
};

static blit::APIData blit_api_data;

namespace blit {
  const APIConst &api = blit_api_const;
  APIData &api_data = blit_api_data;
}

// user funcs
void init();
void render(uint32_t);
void update(uint32_t);

static volatile uint32_t ms_count = 0;
static gptimer_handle_t timer = nullptr;

static bool timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
  // increment millisecond count (alarm will reset timer)
  ms_count++;

  // wake up main task every millisecond
  auto task = (TaskHandle_t)user_ctx;
  BaseType_t hp_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(task, &hp_task_woken);

  return hp_task_woken;
}

static void init_timer() {
  // setup 1us timer
  gptimer_config_t timer_config = {};
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000 * 1000; // 1MHz / 1us (1kHz would have too high divider)

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

  // setup 1ms alarm
  gptimer_alarm_config_t alarm_config = {};
  alarm_config.reload_count = 0;
  alarm_config.alarm_count = 1000; // 1000us == 1ms
  alarm_config.flags.auto_reload_on_alarm = true;

  ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));

  gptimer_event_callbacks_t callbacks = {};
  callbacks.on_alarm = timer_alarm_cb;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &callbacks, xTaskGetCurrentTaskHandle()));

  // start
  ESP_ERROR_CHECK(gptimer_enable(timer));
  ESP_ERROR_CHECK(gptimer_start(timer));
}

static uint32_t now() {
  return ms_count;
}

static void debug(const char *message) {
  auto p = message;
  while(*p)
    putchar(*p++);
}

static bool set_screen_mode_format(blit::ScreenMode new_mode, blit::SurfaceTemplate &new_surf_template) {
  // default format
  if(new_surf_template.format == (blit::PixelFormat)-1)
    new_surf_template.format = blit::PixelFormat::RGB565;

  // default bounds
  switch(new_mode) {
    case blit::ScreenMode::lores:
      if(new_surf_template.bounds.empty())
        new_surf_template.bounds = lores_screen_size;
      else
        new_surf_template.bounds /= 2;

      break;
    case blit::ScreenMode::hires:
    case blit::ScreenMode::hires_palette:
      if(new_surf_template.bounds.empty())
        new_surf_template.bounds = hires_screen_size;

      break;
  }

  if(!display_mode_supported(new_mode, new_surf_template))
    return false;

  // TODO: store info?

  display_mode_changed(new_mode, new_surf_template);

  return true;
}

extern "C"
void app_main() {
  init_timer();
  init_display();

  // FIXME: this should be lores, but that isn't implemented yet
  blit::set_screen_mode(blit::ScreenMode::hires);

  blit::render = ::render;
  blit::update = ::update;

  // user init
  ::init();

  while(true) {
    auto now = ::now();
    update_display(now);

    int ms_to_next_update = blit::tick(::now());

    // more update

    // sleep?

    if(ms_to_next_update > 1 && !display_render_needed()) {
      // wait until timer wakes us up again
      xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);
    }
  }
}
