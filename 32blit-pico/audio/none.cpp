#include "audio.hpp"

#include "audio/audio.hpp"

static uint32_t last_update_time = 0;

void init_audio() {
}

void update_audio(uint32_t time) {
  uint32_t elapsed = time - last_update_time;
  last_update_time = time;

  for(auto f = 0u; f < elapsed * blit::sample_rate / 1000; f++) {
    blit::get_audio_frame();
  }
}
