#include "sdmmc_cmd.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

#include "driver/sdspi_host.h"

#include "storage.hpp"
#include "config.h"

static sdmmc_host_t host;
static sdmmc_card_t card;
static bool host_initialised = false, card_initialised = false;

static void host_init() {

#ifdef SD_SPI
  // int spi bus
  spi_host_device_t spi_slot = SDSPI_DEFAULT_HOST;
  spi_bus_config_t bus_config = {};
  bus_config.mosi_io_num = SD_SPI_MOSI_PIN;
  bus_config.miso_io_num = SD_SPI_MISO_PIN;
  bus_config.sclk_io_num = SD_SPI_SCK_PIN;
  ESP_ERROR_CHECK(spi_bus_initialize(spi_slot, &bus_config, SDSPI_DEFAULT_DMA));

  // init sdspi host
  ESP_ERROR_CHECK(sdspi_host_init());

  sdspi_dev_handle_t sdspi_handle;
  sdspi_device_config_t sdspi_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  sdspi_config.gpio_cs = gpio_num_t(SD_SPI_CS_PIN);
  sdspi_config.host_id = spi_slot;
  ESP_ERROR_CHECK(sdspi_host_init_device(&sdspi_config, &sdspi_handle));

  // prepare to init sdmmc host
  host = SDSPI_HOST_DEFAULT();
  host.slot = spi_slot;
#endif

#ifdef SD_LDO_ID
  // setup LDO
  sd_pwr_ctrl_ldo_config_t ldo_config = {
    .ldo_chan_id = SD_LDO_ID,
  };
  sd_pwr_ctrl_handle_t pwr_ctrl_handle = nullptr;

  ESP_ERROR_CHECK(sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle));
  host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif

  host_initialised = true;
}

bool storage_init() {
  if(!host_initialised)
    host_init();

  // init card
  card_initialised = sdmmc_card_init(&host, &card) == ESP_OK;
  return card_initialised;
}

bool is_storage_available() {
  return card_initialised;
}

bool has_storage_changed() {
  return false;
}

void get_storage_size(uint16_t &block_size, uint32_t &num_blocks) {
  block_size = card.csd.sector_size;
  num_blocks = card.csd.capacity;
}

int32_t storage_read(uint32_t sector, uint32_t offset, void *buffer, uint32_t size_bytes) {
  // offset should be 0
  if(sdmmc_read_sectors(&card, buffer, sector, size_bytes / 512) != ESP_OK)
    return -1;

  return size_bytes;
}

int32_t storage_write(uint32_t sector, uint32_t offset, const uint8_t *buffer, uint32_t size_bytes) {
  if(sdmmc_write_sectors(&card, buffer, sector, size_bytes / 512) != ESP_OK)
    return -1;

  return size_bytes;
}
