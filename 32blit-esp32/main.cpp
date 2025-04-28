#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gptimer.h"

#include "engine/api_private.hpp"

static uint32_t now();

// blit API
static const blit::APIConst blit_api_const {
  blit::api_version_major, blit::api_version_minor,

  nullptr, // channels

  nullptr, // set_screen_mode
  nullptr, // set_screen_palette

  ::now,
  nullptr, // random
  nullptr, // exit
  nullptr, // debug

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

  nullptr, // set_screen_mode_format

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

static gptimer_handle_t timer = nullptr;

static void init_timer() {
  // setup 1ms timer
  gptimer_config_t timer_config = {};
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000 * 1000; // 1MHz / 1us (1kHz would have too high divider)

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));
  ESP_ERROR_CHECK(gptimer_enable(timer));
  ESP_ERROR_CHECK(gptimer_start(timer));
}

static uint32_t now() {
  uint64_t timer_val;
  gptimer_get_raw_count(timer, &timer_val);
  return timer_val / 1000;
}

extern "C"
void app_main() {
  init_timer();

  // set_screen_mode

  blit::render = ::render;
  blit::update = ::update;

  // user init
  ::init();

  uint32_t last_render = 0;

  while(true) {

    // render timing placeholder
    auto render_now = ::now();
    if(render_now - last_render >= 20) {
      ::render(render_now);
      last_render = render_now;
    }

    blit::tick(::now());

    // more update

    // sleep

    vTaskDelay(1); // hmm, this is 10ms
  }
}
