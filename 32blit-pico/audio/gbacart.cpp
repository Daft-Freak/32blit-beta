#include <cmath>

#include "audio.hpp"

#include "audio/audio.hpp"

#include "gbacart.h"

static constexpr int audio_buf_size = 368;

int8_t audio_buf[2][audio_buf_size];

static void fill_audio_buf(int8_t *ptr) {
  for(int i = 0; i < audio_buf_size; i++) {
    int val = (int)blit::get_audio_frame() - 0x8000;

    *ptr++ = val >> 8;
  }
}

void init_audio() {
  auto cart_api = gbacart_get_api();
  cart_api->audio_buf_size = audio_buf_size;
  // TODO: use a sample rate that matches the output and resample?
  cart_api->audio_timer = 0x10000 - std::round((1 << 24) / 22050.0f);
}

void update_audio(uint32_t time) {
  auto cart_api = gbacart_get_api();
  // update audio buffers
  if(!cart_api->audio_addr[0]) {
    fill_audio_buf(audio_buf[0]);
    cart_api->audio_addr[0] = gbacart_to_gba_addr(audio_buf[0]);
  }

  if(!cart_api->audio_addr[1]) {
    fill_audio_buf(audio_buf[1]);
    cart_api->audio_addr[1] = gbacart_to_gba_addr(audio_buf[1]);
  }
}
