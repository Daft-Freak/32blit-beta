#include "stm32h7xx_hal.h"

#include "32blit.hpp"
#include "engine/api_private.hpp"
#include "gpio.hpp"
#include "sound.hpp"

TIM_HandleTypeDef htim6;
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch2;

#define AUDIO_BUFFER_SIZE 512
static uint16_t audio_buffer[AUDIO_BUFFER_SIZE];

static void refill_buffer(uint16_t *ptr, int count) {
  static bool was_amp_enabled = true;
  bool enable_amp = sound::enabled && is_audio_playing();

  if(enable_amp != was_amp_enabled) {
    gpio::write(AMP_SHUTDOWN_GPIO_Port, AMP_SHUTDOWN_Pin, enable_amp);
    was_amp_enabled = enable_amp;
  }

  for(int i = 0; i < count; i++)
    ptr[i] = sound::enabled ? blit::get_audio_frame() >> 4 : 0x800;

  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)ptr, count * 2);
}

static void dma_half_complete_handler(DMA_HandleTypeDef *dma) {
  // refill first half
  refill_buffer(audio_buffer, AUDIO_BUFFER_SIZE / 2);
}

static void dma_complete_handler(DMA_HandleTypeDef *dma) {
  // refill second half
  refill_buffer(audio_buffer + AUDIO_BUFFER_SIZE / 2, AUDIO_BUFFER_SIZE / 2);
}

namespace sound {

  AudioChannel channels[CHANNEL_COUNT];
  bool enabled = true;

  void init() {
    ((APIConst *) &blit::api)->channels = channels;

    // setup the 22,010Hz audio timer
    __TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 310;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 34;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
      // TODO: fail
    }

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
      // TODO: fail
    }

    __HAL_RCC_DAC12_CLK_ENABLE();
    DAC_ChannelConfTypeDef sConfig = {0};

    // setup the dac output
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK)
    {
      // TODO: fail
    }
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
      // TODO: fail
    }

    // setup DMA
    hdma_dac1_ch2.Instance = DMA1_Stream2;
    hdma_dac1_ch2.Init.Request = DMA_REQUEST_DAC1_CH2;
    hdma_dac1_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac1_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac1_ch2.Init.Mode = DMA_CIRCULAR;
    hdma_dac1_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dac1_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dac1_ch2) != HAL_OK)
    {
      // TODO: fail
    }

    __HAL_LINKDMA(&hdac1, DMA_Handle2, hdma_dac1_ch2);

    hdma_dac1_ch2.XferHalfCpltCallback = dma_half_complete_handler;
    hdma_dac1_ch2.XferCpltCallback = dma_complete_handler;

    SET_BIT(hdac1.Instance->CR, DAC_CR_DMAEN2);

    // start
    HAL_DMA_Start_IT(&hdma_dac1_ch2, (uintptr_t)audio_buffer, (uintptr_t)&hdac1.Instance->DHR12R2, AUDIO_BUFFER_SIZE);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  }

}


