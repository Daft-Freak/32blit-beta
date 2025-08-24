#include <array>

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio.hpp"
#include "config.h"

#include "audio/audio.hpp"

#ifndef AUDIO_I2S_SHIFT
#define AUDIO_I2S_SHIFT 0
#endif

static i2s_chan_handle_t i2s_handle = nullptr;

static void audio_task(void *) {
  uint16_t samples[256];
  while(true) {
    for(size_t i = 0; i < std::size(samples); i += 2)
      samples[i] = samples[i + 1] = ((int)blit::get_audio_frame() - 0x8000) >> AUDIO_I2S_SHIFT;

    i2s_channel_write(i2s_handle, &samples, sizeof(samples), nullptr, portMAX_DELAY);
  }
}

void init_audio() {
#ifdef AUDIO_I2S_MUTE_PIN
  // configure mute pin
  gpio_config_t mute_gpio_config = {};
  mute_gpio_config.mode = GPIO_MODE_OUTPUT;
  mute_gpio_config.pin_bit_mask = 1ULL << AUDIO_I2S_MUTE_PIN;

  ESP_ERROR_CHECK(gpio_config(&mute_gpio_config));
  gpio_set_level(gpio_num_t(AUDIO_I2S_MUTE_PIN), 1);
#endif

  // init channel
  i2s_chan_config_t chan_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chan_config.dma_frame_num = 512;
  ESP_ERROR_CHECK(i2s_new_channel(&chan_config, &i2s_handle, nullptr));

  i2s_std_config_t std_config = {};
  std_config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100);
  std_config.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);

  std_config.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  std_config.gpio_cfg.bclk = gpio_num_t(AUDIO_I2S_BCLK_PIN);
  std_config.gpio_cfg.ws = gpio_num_t(AUDIO_I2S_LRCLK_PIN);
  std_config.gpio_cfg.dout = gpio_num_t(AUDIO_I2S_DATA_PIN);
  std_config.gpio_cfg.din = I2S_GPIO_UNUSED;

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_handle, &std_config));

  // enable
  ESP_ERROR_CHECK(i2s_channel_enable(i2s_handle));

  // create task
  xTaskCreate(audio_task, "i2s", 2048, nullptr, 5, nullptr);
}

void update_audio(uint32_t time) {
  // handled by the task
  // which is good because this doesn't get called very frequently
}
