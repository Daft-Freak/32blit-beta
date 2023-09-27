#include <cstring>

#include "libretro.h"

#include "32blit.hpp"

#include "api.hpp"
#include "consts.hpp"

static blit::Size screen_bounds;
static blit::PixelFormat screen_format;
static uint8_t *screen_data;

// annoyingly none of our formats are an exact match
// (R/B swapped for RGBA/RGB565, no RGB24)
static uint8_t conv_fb[screen_width * screen_height * 4];

static uint32_t current_time;

// callbacks
static retro_environment_t environment_cb;
static retro_video_refresh_t video_cb;
static retro_audio_sample_t audio_sample_cb;
static retro_input_poll_t input_poll_cb;
static retro_input_state_t input_state_cb;
static retro_log_printf_t log_printf_cb;

static void fallback_log(enum retro_log_level level, const char *fmt, ...) {
  static const char *levels[]{"debug", "info", "warn", "error"};
  fprintf(stderr, "%s: ", levels[level]);

  va_list va;
  va_start(va, fmt);
  vfprintf(stderr, fmt, va);
  va_end(va);
}

void retro_set_environment(retro_environment_t cb) {
  environment_cb = cb;

  // get logging callback
  struct retro_log_callback logging;

  if(cb(RETRO_ENVIRONMENT_GET_LOG_INTERFACE, &logging))
    log_printf_cb = logging.log;
  else
    log_printf_cb = fallback_log;

  // setup controller
  static const struct retro_controller_description controllers[] = {
    {"32blit", RETRO_DEVICE_SUBCLASS(RETRO_DEVICE_JOYPAD, 0)},
  };

  static const struct retro_controller_info ports[] = {
    {controllers, 1},
    {nullptr, 0},
  };

  cb(RETRO_ENVIRONMENT_SET_CONTROLLER_INFO, (void*)ports);

  // can run with no file, most things only do that
  bool no_game = true;
  cb(RETRO_ENVIRONMENT_SET_SUPPORT_NO_GAME, &no_game);
}

void retro_set_video_refresh(retro_video_refresh_t cb) {
  video_cb = cb;
}

void retro_set_audio_sample(retro_audio_sample_t cb) {
  audio_sample_cb = cb;
}

void retro_set_audio_sample_batch(retro_audio_sample_batch_t cb) {
  
}

void retro_set_input_poll(retro_input_poll_t cb) {
  input_poll_cb = cb;
}

void retro_set_input_state(retro_input_state_t cb) {
  input_state_cb = cb;
}

// init/deint/info
void retro_init() {
  const char *sys_dir = nullptr, *save_dir = nullptr;
  if(!environment_cb(RETRO_ENVIRONMENT_GET_SYSTEM_DIRECTORY, &sys_dir) || !sys_dir)
    sys_dir = "";

  if(!environment_cb(RETRO_ENVIRONMENT_GET_SAVE_DIRECTORY, &save_dir) || !save_dir)
    save_dir = sys_dir;

	blit::update = ::update;
	blit::render = ::render;

  blit_api_init(sys_dir, save_dir);
}

void retro_deinit() {

}

unsigned retro_api_version() {
  return RETRO_API_VERSION;
}

void retro_get_system_info(struct retro_system_info *info) {
  memset(info, 0, sizeof(*info));
  info->library_name     = metadata_title;
  info->library_version  = metadata_version;
  info->need_fullpath    = true;
  info->valid_extensions = ""; // TODO: from metadata
}

void retro_get_system_av_info(struct retro_system_av_info *info) {
  info->geometry.base_width   = screen_width;
  info->geometry.base_height  = screen_height;
  info->geometry.max_width    = screen_width;
  info->geometry.max_height   = screen_height;
  info->geometry.aspect_ratio = float(screen_width) / screen_height;

  info->timing.fps = fps;
  info->timing.sample_rate = sample_rate;
}

void retro_set_controller_port_device(unsigned port, unsigned device) {
}

void retro_reset() {
  // can't really do this...
}

void retro_run() {
  input_poll_cb();

  // input
  auto get_button = [](int retro_id, blit::Button blit_button){
    return input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, retro_id) ? uint32_t(blit_button) : 0;
  };

  blit::buttons = get_button(RETRO_DEVICE_ID_JOYPAD_LEFT  , blit::Button::DPAD_LEFT)
                | get_button(RETRO_DEVICE_ID_JOYPAD_RIGHT , blit::Button::DPAD_RIGHT)
                | get_button(RETRO_DEVICE_ID_JOYPAD_UP    , blit::Button::DPAD_UP)
                | get_button(RETRO_DEVICE_ID_JOYPAD_DOWN  , blit::Button::DPAD_DOWN)
                | get_button(RETRO_DEVICE_ID_JOYPAD_A     , blit::Button::A)
                | get_button(RETRO_DEVICE_ID_JOYPAD_B     , blit::Button::B)
                | get_button(RETRO_DEVICE_ID_JOYPAD_X     , blit::Button::X)
                | get_button(RETRO_DEVICE_ID_JOYPAD_Y     , blit::Button::Y)
                | get_button(RETRO_DEVICE_ID_JOYPAD_SELECT, blit::Button::MENU)
                | get_button(RETRO_DEVICE_ID_JOYPAD_START , blit::Button::HOME)
                | get_button(RETRO_DEVICE_ID_JOYPAD_L3    , blit::Button::JOYSTICK);

  blit::joystick.x = float(input_state_cb(0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_X)) / 0x7FFF;
  blit::joystick.y = float(input_state_cb(0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_Y)) / 0x7FFF;

  // update/render
  blit::render(current_time);

  blit::tick(current_time);

  // convert screen data
  switch(screen_format) {
    case blit::PixelFormat::RGB565: {
      auto in16 = (uint16_t *)screen_data;
      auto out16 = (uint16_t *)conv_fb;
      for(int i = 0; i < screen_bounds.area(); i++) {
        *out16++ = (*in16 & 0x07E0) | (*in16 << 11) | (*in16 >> 11);
        in16++;
      }
      break;
    }

    case blit::PixelFormat::RGB: {
      auto out32 = (uint32_t *)conv_fb;
      for(int i = 0; i < screen_bounds.area(); i++) {
        *out32++ = screen_data[i * 3 + 0] << 16
                 | screen_data[i * 3 + 1] <<  8
                 | screen_data[i * 3 + 2] <<  0;
      }
      break;
    }

    default:
      break; // should be unreachable
  }

  int format_size = screen_format == blit::PixelFormat::RGB565 ? 2 : 4;
  video_cb(conv_fb, screen_bounds.w, screen_bounds.h, screen_bounds.w * format_size);

  // audio
  for(int i = 0; i < samples_per_frame; i++) {
    int sample = blit::get_audio_frame() - 0x8000;
    audio_sample_cb(sample, sample);
  }

  // we're supposed to run a frame here, so fake time
  current_time += 20;
}

// save states
size_t retro_serialize_size() {
  return 0;
}

bool retro_serialize(void *data, size_t size) {
  return false;
}

bool retro_unserialize(const void *data, size_t size) {
  return false;
}

// cheats
void retro_cheat_reset() {
}

void retro_cheat_set(unsigned index, bool enabled, const char *code) {
}

// file load/unload
bool retro_load_game(const struct retro_game_info *game) {

  struct retro_input_descriptor desc[] = {
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT,   "Left"            },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP,     "Up"              },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN,   "Down"            },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT,  "Right"           },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A,      "A"               },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B,      "B"               },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X,      "X"               },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_Y,      "Y"               },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START,  "Home"            },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_SELECT, "Menu"            },
    {0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L3,     "Joystick Button" },
  
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_X, "Joystick X" },
    {0, RETRO_DEVICE_ANALOG, RETRO_DEVICE_INDEX_ANALOG_LEFT, RETRO_DEVICE_ID_ANALOG_Y, "Joystick Y" },

    {0, 0, 0, 0, nullptr},
  };

  environment_cb(RETRO_ENVIRONMENT_SET_INPUT_DESCRIPTORS, desc);

  // TODO: audio cb?

  // init game
  blit::set_screen_mode(blit::ScreenMode::lores);

  ::init();

  return true;
}

bool retro_load_game_special(unsigned game_type, const struct retro_game_info *info, size_t num_info) {
  return false;
}

void retro_unload_game() {
  // can't do this either
}

// region
unsigned retro_get_region() {
  return RETRO_REGION_PAL;
}

// memory
void *retro_get_memory_data(unsigned id) {
  return nullptr;
}

size_t retro_get_memory_size(unsigned id) {
  return 0;
}

// 32blit api handlers
bool screen_mode_changed(blit::SurfaceTemplate &info) {
  auto retro_format = RETRO_PIXEL_FORMAT_XRGB8888;

  switch(info.format) {
    case blit::PixelFormat::RGB:
      break; // already set
    case blit::PixelFormat::RGB565:
      retro_format = RETRO_PIXEL_FORMAT_RGB565;
      break;

    default:
      return false;
  }

  if(!environment_cb(RETRO_ENVIRONMENT_SET_PIXEL_FORMAT, &retro_format))
    return false;

  retro_game_geometry geom = {};

  geom.base_width = info.bounds.w;
  geom.base_height = info.bounds.h;
  geom.aspect_ratio = float(info.bounds.w) / info.bounds.h;

  if(!environment_cb(RETRO_ENVIRONMENT_SET_GEOMETRY, &geom))
    return false;

  screen_bounds = info.bounds;
  screen_format = info.format;
  screen_data = info.data;
  
  return true;
}

uint32_t get_current_time() {
  return current_time;
}

void debug_message(const char *message) {
  log_printf_cb(RETRO_LOG_DEBUG, "%s", message);
}