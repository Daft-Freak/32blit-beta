#pragma once

#ifndef ALLOW_HIRES
#define ALLOW_HIRES 0 // disable by default, mode switching isn't supported
#endif

#define AUDIO_MAX_SAMPLE_UPDATE 64
#define AUDIO_I2S_CLOCK_PIN_BASE 27
#define AUDIO_I2S_DATA_PIN 26

// native
#define SD_CLK   5
#define SD_CMD  18
#define SD_DAT0 19

// spi
#define SD_SCK   5
#define SD_MOSI 18
#define SD_MISO 19
#define SD_CS   22
