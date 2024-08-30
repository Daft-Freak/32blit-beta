#include <array>

#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

#include "audio.hpp"
#include "config.h"

#include "audio/audio.hpp"

#define AUDIO_FREQ 22050
#define AUDIO_BITS 10
#define BUFFER_SIZE 512

static unsigned int audio_pwm_slice;
static unsigned int dma_channel;

static uint16_t buffer_data[BUFFER_SIZE * 2];
static volatile bool need_refill = true;
static volatile uint16_t *dma_buffer = nullptr, *refill_buffer = buffer_data;

static void dma_irq_handler() {
  if(dma_channel_get_irq0_status(dma_channel)) {
    dma_channel_acknowledge_irq0(dma_channel);

    if(!need_refill) {
      std::swap(dma_buffer, refill_buffer);
      need_refill = true;
    }

    dma_channel_set_read_addr(dma_channel, dma_buffer, true);
  }
}

void init_audio() {
  // TODO: config
  // we can't use the other half of the slice for anything (DMA writes both counters)
  gpio_set_function(AUDIO_BEEP_PIN, GPIO_FUNC_PWM);

  audio_pwm_slice = pwm_gpio_to_slice_num(AUDIO_BEEP_PIN);
  const int wrap = (1 << AUDIO_BITS) - 1;

  int clkdiv = (clock_get_hz(clk_sys) * 16) / (AUDIO_FREQ * (1 << AUDIO_BITS));

  pwm_set_clkdiv_int_frac(audio_pwm_slice, clkdiv >> 4, clkdiv & 0xF);
  pwm_set_wrap(audio_pwm_slice, wrap);

  //dma
  dma_channel = dma_claim_unused_channel(true);
  auto config = dma_channel_get_default_config(dma_channel);
  channel_config_set_read_increment(&config, true);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
  channel_config_set_dreq(&config, DREQ_PWM_WRAP0 + audio_pwm_slice);

  dma_channel_set_irq0_enabled(dma_channel, true);
  irq_add_shared_handler(DMA_IRQ_0, dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY + 1);
  irq_set_enabled(DMA_IRQ_0, true);

  dma_channel_configure(dma_channel, &config, &pwm_hw->slice[audio_pwm_slice].cc, nullptr, BUFFER_SIZE, false);
}

void update_audio(uint32_t time) {

  if(need_refill) {
    for(int i = 0; i < BUFFER_SIZE; i++) {
      refill_buffer[i] = blit::get_audio_frame() >> (16 - AUDIO_BITS);
    }

    need_refill = false;

    if(!dma_buffer) {
      // first buffer, start dma
      dma_buffer = refill_buffer;
      refill_buffer += BUFFER_SIZE;
      dma_channel_set_read_addr(dma_channel, dma_buffer, true);
      pwm_set_enabled(audio_pwm_slice, true);
      // immediately start 2nd buffer
      need_refill = true;
    }
  }
}
