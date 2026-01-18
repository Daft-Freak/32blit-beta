#include <cstdint>

#include "core_pins.h"
#include "smalloc.h"
#include "usb_serial.h"

#include "engine/engine.hpp"
#include "engine/api_private.hpp"

#include "display.hpp"

using namespace blit;

// HACK: stub EXTMEM allocator
int sm_set_pool(struct smalloc_pool *, void *, size_t, int, smalloc_oom_handler) {
  return 0;
}

static blit::AudioChannel channels[CHANNEL_COUNT];

// blit API
static const blit::APIConst blit_api_const {
  blit::api_version_major, blit::api_version_minor,

  ::channels,

  nullptr, // set_screen_mode
  nullptr, // set_screen_palette

  millis, // now
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

  display::set_screen_mode_format,

  nullptr, // i2c_send
  nullptr, // i2c_recieve

  nullptr, // set_raw_cdc_enabled
  nullptr, // cdc_write
  nullptr, // cdc_read

  nullptr, // list_installed_games
  nullptr, // can_launch
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

int main() {
  Serial.begin(9600); //dbg

  display::init();

  ::set_screen_mode(ScreenMode::lores);

  blit::render = ::render;
  blit::update = ::update;

  // user init
  ::init();

  while(true) {
    tick(millis());

    auto now = millis();

    if(display::update_needed(now)) {
      ::render(now);
      display::update(now);
    }
  }

  return 0;
}
