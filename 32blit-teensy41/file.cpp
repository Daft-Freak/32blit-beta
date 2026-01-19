#include <cstdint>

#include "core_pins.h" // for usb_serial
#include "usb_serial.h" // Serial.printf

#include "ff.h"
#include "diskio.h"

#include "file.hpp"
#include "sd.hpp"

static FATFS fs;

// fatfs io funcs
DSTATUS disk_initialize(BYTE pdrv) {
  // should have already initialised card before calling init
  return disk_status(pdrv);
}

DSTATUS disk_status(BYTE pdrv) {
  return sd_get_initialised() ? RES_OK : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
  static_assert(FF_MIN_SS == FF_MAX_SS);
  return sd_read_blocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
  return sd_write_blocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  switch(cmd) {
    case CTRL_SYNC:
      return RES_OK;

    case GET_SECTOR_COUNT:
      *(LBA_t *)buff = sd_get_num_blocks();
      return RES_OK;

    case GET_BLOCK_SIZE:
      *(DWORD *)buff = 1;
      return RES_OK;
  }

  return RES_PARERR;
}

void init_fs() {
  auto res = f_mount(&fs, "", 1);

  if(res != FR_OK)
    Serial.printf("Failed to mount filesystem! (%i)\n", res);
}
