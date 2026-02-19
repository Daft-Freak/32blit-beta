#pragma once

//#define AUDIO_MAX_SAMPLE_UPDATE 64
#define AUDIO_I2S_CLOCK_PIN_BASE 0
#define AUDIO_I2S_DATA_PIN 2
#define AUDIO_I2S_PIO 1 // PIO 0 is used for the display and needs to set IO base


#define DPI_DATA_PIN_BASE 21
// uhoh, this won't work
//#define DPI_SYNC_PIN_BASE 6
#define DPI_SYNC_PIN_BASE 19

// native
#define SD_CLK  41
#define SD_CMD  42
#define SD_DAT0 37

// spi
#define SD_SCK  41
#define SD_MOSI 42
#define SD_MISO 37
#define SD_CS   40

#define SD_PIO 0 // needs IO base like display

#define SD_SPI_OVERCLOCK 0 // seem to have some problems
