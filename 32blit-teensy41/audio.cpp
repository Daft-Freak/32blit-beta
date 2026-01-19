#include <array>
#include <cstdint>
#include <cstring>

#include "avr/pgmspace.h" // DMAMEM
#include "core_pins.h"
#include "imxrt.h"
#include "DMAChannel.h"

#include "audio/audio.hpp"

#include "audio.hpp"

namespace audio {

  DMAMEM static int16_t audio_buffer[256 * 2] alignas(32);

  static DMAChannel audio_dma_channel(false);

  static void refill_audio_buffer(int16_t *buf_ptr, unsigned int len) {
    for(size_t i = 0; i < len; i += 4) {
      int sample = int(blit::get_audio_frame()) - 0x8000;
      *buf_ptr++ = sample;
      *buf_ptr++ = sample;
      *buf_ptr++ = sample;
      *buf_ptr++ = sample;
    }
  }

  static void audio_dma_interrupt() {
    audio_dma_channel.clearInterrupt();

    auto dma_addr = (int16_t *)audio_dma_channel.TCD->SADDR;

    // refill the half that just finished (the one the dma channel isn't reading)
    auto half_size = std::size(audio_buffer) / 2;
    bool refill_first_half = dma_addr >= audio_buffer + half_size;
    auto cur_ptr = refill_first_half ? audio_buffer : audio_buffer + half_size;

    refill_audio_buffer(cur_ptr, half_size);
    arm_dcache_flush_delete(cur_ptr, half_size * sizeof(int16_t)); // flush cache
  }

  void init() {
    // setup PLL
    // 24MHz ref
    // 27 + 6/125 == 27.048
    // 24MHz * 27.048 / 2 == 324.576MHz
    CCM_ANALOG_PLL_AUDIO = CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(1) // 0 = /4, 1 = /2, 2 = /1
                         | CCM_ANALOG_PLL_AUDIO_ENABLE
                         | CCM_ANALOG_PLL_AUDIO_DIV_SELECT(27); // valid range: 27 - 54

    // fraction
    CCM_ANALOG_PLL_AUDIO_NUM = 6;
    CCM_ANALOG_PLL_AUDIO_DENOM = 125;

    // lock
    while (!(CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_LOCK));

    CCM_ANALOG_MISC2 &= ~(CCM_ANALOG_MISC2_DIV_MSB | CCM_ANALOG_MISC2_DIV_LSB); // another divider

    // setup SAI2 clocks
    CCM_CCGR5 |= CCM_CCGR5_SAI2(CCM_CCGR_ON);
    CCM_CSCMR1 = (CCM_CSCMR1 & ~CCM_CSCMR1_SAI2_CLK_SEL_MASK) | CCM_CSCMR1_SAI2_CLK_SEL(2); // PLL4/audio PLL

    // 324.576MHz / 5 / 23 == 2.8224MHz
    uint32_t mask = CCM_CS2CDR_SAI2_CLK_PRED_MASK | CCM_CS2CDR_SAI2_CLK_PODF_MASK;
    CCM_CS2CDR = (CCM_CS2CDR & ~mask) | CCM_CS2CDR_SAI2_CLK_PRED(5 - 1) | CCM_CS2CDR_SAI2_CLK_PODF(23 - 1);

    // pins (ALT2)
    CORE_PIN2_CONFIG = 2; // DATA
    CORE_PIN3_CONFIG = 2; // LRCLK
    CORE_PIN4_CONFIG = 2; // BCLK

    // configure SAI2
    // 2.8224MHz / 2 == 1.4112MHz
    // == 44100Hz * 16 * 2

    const int sample_bits = 16;

    I2S2_TCR1 = I2S_TCR1_RFW(1);                 // FIFO watermark

    I2S2_TCR2 = I2S_TCR2_SYNC(0)                 // asynchronous
              | I2S_TCR2_MSEL(1)                 // MCLK sel
              | I2S_TCR2_BCP                     // active low
              | I2S_TCR2_BCD                     // generate internally
              | I2S_TCR2_DIV(0);                 // /2
    I2S2_TCR3 = I2S_TCR3_TCE;                    // enable first channel

    I2S2_TCR4 = I2S_TCR4_FRSZ((2 - 1))           // 2 words in frame
              | I2S_TCR4_SYWD((sample_bits - 1)) // sync width
              | I2S_TCR4_MF                      // MSB first
              | I2S_TCR4_FSE                     // sync early
              | I2S_TCR4_FSP                     // active high sync
              | I2S_TCR4_FSD;                    // generate sync internally

    I2S2_TCR5 = I2S_TCR5_WNW((sample_bits - 1))  // word width
              | I2S_TCR5_W0W((sample_bits - 1))  // first word width
              | I2S_TCR5_FBT((sample_bits - 1)); // first bit


    memset(audio_buffer, 0, sizeof(audio_buffer));

    // setup DMA
    audio_dma_channel.begin();

    audio_dma_channel.sourceBuffer(audio_buffer, sizeof(audio_buffer));
    audio_dma_channel.destination(I2S2_TDR0);
    audio_dma_channel.TCD->ATTR_DST = 1;// 16 bit writes
    audio_dma_channel.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI2_TX);

    audio_dma_channel.interruptAtCompletion();
    audio_dma_channel.interruptAtHalf();
    audio_dma_channel.attachInterrupt(audio_dma_interrupt);

    // enable
    audio_dma_channel.enable();
    I2S2_TCSR = I2S_TCSR_TE | I2S_TCSR_FRDE;
  }
}
