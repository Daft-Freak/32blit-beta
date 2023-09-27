#include <random>

#include "api.hpp"
#include "consts.hpp"

using namespace blit;

static blit::AudioChannel channels[CHANNEL_COUNT];

static auto default_screen_format = blit::PixelFormat::RGB;
static SurfaceInfo cur_surf_info;
static constexpr Size hires_screen_size(screen_width, screen_height);
static constexpr Size lores_screen_size(screen_width / 2, screen_height / 2);

static uint8_t screen_fb[screen_width * screen_height * 4];

static bool set_screen_mode_format(ScreenMode new_mode, SurfaceTemplate &new_surf_template) {
  new_surf_template.data = screen_fb;

  if(new_surf_template.format == (PixelFormat)-1)
    new_surf_template.format = default_screen_format;

  switch(new_mode) {
    case ScreenMode::lores:
      new_surf_template.bounds = lores_screen_size;
      break;
    case ScreenMode::hires:
    case ScreenMode::hires_palette:
      new_surf_template.bounds = hires_screen_size;
      break;
  }

  if(!screen_mode_changed(new_surf_template))
    return false;

  return true;
}

void set_screen_palette(const Pen *colours, int num_cols) {

}

// blit random callback
#ifdef __MINGW32__
// Windows/MinGW doesn't support a non-deterministic source of randomness, so we fall back upon the age-old time seed once more
// Without this, random_device() will always return the same number and thus our mersenne twister will always produce the same sequence.
static std::mt19937 random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
#else
static std::random_device random_device;
static std::mt19937 random_generator(random_device());
#endif
static std::uniform_int_distribution<uint32_t> random_distribution;
static uint32_t blit_random() {
  return random_distribution(random_generator);
}


void blit_api_init() {
  api.channels = ::channels;

  api.set_screen_mode = nullptr;
  api.set_screen_palette = ::set_screen_palette;
  api.set_screen_mode_format = ::set_screen_mode_format;
  api.now = ::get_current_time;
  api.random = ::blit_random;
  // api.exit = ::exit;

  // serial debug
  // api.debug = ::debug;

  // files
  // api.open_file = ::open_file;
  // api.read_file = ::read_file;
  // api.write_file = ::write_file;
  // api.close_file = ::close_file;
  // api.get_file_length = ::get_file_length;
  // api.list_files = ::list_files;
  // api.file_exists = ::file_exists;
  // api.directory_exists = ::directory_exists;
  // api.create_directory = ::create_directory;
  // api.rename_file = ::rename_file;
  // api.remove_file = ::remove_file;
  // api.get_save_path = ::get_save_path;
  // api.is_storage_available = ::is_storage_available;

  // profiler
  // api.enable_us_timer = ::enable_us_timer;
  // api.get_us_timer = ::get_us_timer;
  // api.get_max_us_timer = ::get_max_us_timer;

  // jpeg
  // api.decode_jpeg_buffer = ::decode_jpeg_buffer;
  // api.decode_jpeg_file = ::decode_jpeg_file;

  // launcher
  // api.launch = ::launch;
  // api.erase_game = ::erase_game;
  // api.get_type_handler_metadata = ::get_type_handler_metadata;

  // api.get_launch_path = ::get_launch_path;

  // multiplayer
  // api.is_multiplayer_connected = ::is_multiplayer_connected;
  // api.set_multiplayer_enabled = ::set_multiplayer_enabled;
  // api.send_message = ::send_multiplayer_message;

  // api.flash_to_tmp = ::flash_to_tmp;
  // api.tmp_file_closed = ::tmp_file_closed;

  // api.get_metadata = ::get_metadata;
}
