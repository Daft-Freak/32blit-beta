#include <cstdint>

#include "avr/pgmspace.h"
#include "core_pins.h"
#include "imxrt.h"
#include "pins_arduino.h"

#include "usb_serial.h"

#include "display.hpp"
#include "display_commands.hpp"

using namespace blit;

enum ST7789Reg {
  RAMCTRL   = 0xB0,
  PORCTRL   = 0xB2,
  GCTRL     = 0xB7,
  VCOMS     = 0xBB,
  LCMCTRL   = 0xC0,
  VDVVRHEN  = 0xC2,
  VRHS      = 0xC3,
  VDVS      = 0xC4,
  FRCTRL2   = 0xC6,
  PWCTRL1   = 0xD0,
  PVGAMCTRL = 0xE0,
  NVGAMCTRL = 0xE1,
};

namespace display {
  DMAMEM static uint8_t screen_fb[320 * 240 * 3]; // possibly EXTMEM

  static const blit::Size lores_screen_size(160, 120);
  static const blit::Size hires_screen_size(320, 240);

  ScreenMode cur_screen_mode = ScreenMode::lores;

  static const int flexIOToPin[]{19, 18, 14, 15, 40, 41, 17, 16,
                                 22, 23, 20, 21, 38, 39, 26, 27}; //0-15, 16-19 and 28-29 are also available

  // flexio pins
  static const int data0FlexPin = 0; // 0-7
  static const int rdFlexPin = 8;
  static const int wrFlexPin = 9;

  // control pins, all active low
  static const int csPin = 21;
  static const int dcPin = 20;
  static const int resetPin = 13;

  // helpers
  static void select() {
    digitalWriteFast(csPin, 0);
  }

  static void deselect() {
    digitalWriteFast(csPin, 1);
  }

  static void command() {
    digitalWriteFast(dcPin, 0);
  }

  static void data() {
    digitalWriteFast(dcPin, 1);
  }

  /*static void read_active() {
    digitalWriteFast(flexIOToPin[rdFlexPin], 0);
  }*/

  static void read_idle() {
    digitalWriteFast(flexIOToPin[rdFlexPin], 1);
  }

  /*static void set_read_mode() {
    for(int i = 0; i < 8; i++)
      pinMode(flexIOToPin[data0FlexPin + i], INPUT);

    pinMode(flexIOToPin[wrFlexPin], OUTPUT);
  }*/

  static void set_write_mode() {
    for(int i = 0; i < 8; i++)
      *portConfigRegister(flexIOToPin[data0FlexPin + i]) = 0x19;

    *portConfigRegister(flexIOToPin[wrFlexPin]) = 0x19;
    pinMode(flexIOToPin[rdFlexPin], OUTPUT);
  }

  /*static uint8_t read8() {
    read_active();
    delayMicroseconds(1); // ?

    uint8_t ret = 0;
    for(int i = 0; i < 8; i++) {
      if(digitalReadFast(dataPins[i]))
        ret |= 1 << i;
    }

    read_idle();
    return ret;
  }*/

  static void write8(uint8_t v) {
    FLEXIO3_TIMSTAT |= (1 << 0);
    FLEXIO3_SHIFTBUF0 = v;

    // wait
    while(!(FLEXIO3_TIMSTAT & (1 << 0)));
  }

  static void command(uint8_t command, size_t len = 0, const char *data = nullptr) {
    select();

    digitalWriteFast(dcPin, 0); // command mode
    write8(command);

    if(data) {
      digitalWriteFast(dcPin, 1); // data mode

      for(size_t i = 0; i < len; i++)
        write8(data[i]);
    }

    deselect();
  }

  static void set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint32_t cols = __builtin_bswap32((x << 16) | (x + w - 1));
    uint32_t rows = __builtin_bswap32((y << 16) | (y + h - 1));

    command(MIPIDCS::SetColumnAddress, 4, (const char *)&cols);
    command(MIPIDCS::SetRowAddress, 4, (const char *)&rows);
  }

  static void send_init_sequence() {
#ifdef LCD_ILI9431

    // power control 1
    command(0xC0, 1, "\x23"); //4.6v, default 4.5v(0x21)

    // VCOM control 1
    command(0xC0, 2, "\x2B\x2B"); // 3.775v, -1.425v, default 3.925v(0x31), -1.0v (0x3C)

    uint8_t madctl = MADCTL::ROW_ORDER | MADCTL::COL_ORDER | MADCTL::SWAP_XY;
    command(MIPIDCS::SetAddressMode, 1, (char *)&madctl);
#else // ST7789
    // 320x240
    command(ST7789Reg::PORCTRL, 5, "\x0c\x0c\x00\x33\x33");
    command(ST7789Reg::GCTRL, 1, "\x35");
    command(ST7789Reg::VCOMS, 1, "\x1f");
    command(ST7789Reg::LCMCTRL, 1, "\x2c");
    command(ST7789Reg::VDVVRHEN, 1, "\x01");
    command(ST7789Reg::VRHS, 1, "\x12");
    command(ST7789Reg::VDVS, 1, "\x20");
    command(ST7789Reg::PWCTRL1, 2, "\xa4\xa1");
    command(0xd6, 1, "\xa1"); // ???
    command(ST7789Reg::PVGAMCTRL, 14, "\xD0\x08\x11\x08\x0C\x15\x39\x33\x50\x36\x13\x14\x29\x2D");
    command(ST7789Reg::NVGAMCTRL, 14, "\xD0\x08\x10\x08\x06\x06\x39\x44\x51\x0B\x16\x14\x2F\x31");

    command(ST7789Reg::FRCTRL2, 1, "\x15"); // 50Hz

    command(MIPIDCS::EnterInvertMode);   // set inversion mode

    uint8_t madctl = MADCTL::SCAN_ORDER | MADCTL::SWAP_XY | MADCTL::ROW_ORDER; //270deg rotation
    command(MIPIDCS::SetAddressMode, 1, (char *)&madctl);
#endif

    command(MIPIDCS::SetPixelFormat, 1, "\x05"); // 16bpp

    command(MIPIDCS::ExitSleepMode);
    delay(120);

    command(MIPIDCS::DisplayOn);

    set_window(0, 0, 320, 240);
  }

  void init() {
    // setup clock
    CCM_CS1CDR = (CCM_CS1CDR & ~CCM_CS1CDR_FLEXIO2_CLK_PODF(7)) | CCM_CS1CDR_FLEXIO2_CLK_PODF(2); // 3
    CCM_CCGR7 |= CCM_CCGR7_FLEXIO3(CCM_CCGR_ON);

    // reset flexio
    FLEXIO3_CTRL &= ~FLEXIO_CTRL_FLEXEN;
    FLEXIO3_CTRL |= FLEXIO_CTRL_SWRST;
    FLEXIO3_CTRL &= ~FLEXIO_CTRL_SWRST;

    // pins
    for(int i = 0; i < 8; i++)
        *portConfigRegister(flexIOToPin[data0FlexPin + i]) = 0x19;

    uint32_t shiftCfg = FLEXIO_SHIFTCFG_PWIDTH(7) | FLEXIO_SHIFTCFG_INSRC;

    FLEXIO3_SHIFTCFG0 = shiftCfg;
    FLEXIO3_SHIFTCTL0 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(3 /*output*/) | FLEXIO_SHIFTCTL_PINSEL(data0FlexPin)
                      | FLEXIO_SHIFTCTL_SMOD(2 /*transmit*/);

    FLEXIO3_SHIFTCFG1 = shiftCfg;
    FLEXIO3_SHIFTCTL1 = FLEXIO_SHIFTCTL_SMOD(2 /*transmit*/);
    FLEXIO3_SHIFTCFG2 = shiftCfg;
    FLEXIO3_SHIFTCTL2 = FLEXIO_SHIFTCTL_SMOD(2 /*transmit*/);
    FLEXIO3_SHIFTCFG3 = shiftCfg;
    FLEXIO3_SHIFTCTL3 = FLEXIO_SHIFTCTL_SMOD(2 /*transmit*/);

    //timcmp cfg ctl
    const int baudDiv = 4; //?
    FLEXIO3_TIMCMP0 = ((1 /*beats*/ * 2 - 1) << 8) | (baudDiv / 2 - 1);
    FLEXIO3_TIMCFG0 = FLEXIO_TIMCFG_TIMDIS(2 /*on compare*/) | FLEXIO_TIMCFG_TIMENA(2/*on trigger high*/);
    FLEXIO3_TIMCTL0 = FLEXIO_TIMCTL_TRGSEL((0 << 2) | 1 /*status flag*/) | FLEXIO_TIMCTL_TRGPOL
                    | FLEXIO_TIMCTL_TRGSRC | FLEXIO_TIMCTL_PINCFG(3 /*output*/) | FLEXIO_TIMCTL_PINSEL(wrFlexPin)
                    | FLEXIO_TIMCTL_PINPOL | FLEXIO_TIMCTL_TIMOD(1 /*dual 8-bit baud*/);

    //enable
    FLEXIO3_CTRL |= FLEXIO_CTRL_FLEXEN;

    pinMode(csPin, OUTPUT);
    pinMode(dcPin, OUTPUT);
    pinMode(flexIOToPin[rdFlexPin], OUTPUT);

    pinMode(resetPin, OUTPUT);

    set_write_mode();

    // reset
    deselect();
    read_idle();

    digitalWriteFast(resetPin, 0);
    delay(1); // ?
    digitalWriteFast(resetPin, 1);

    delay(10);

    // send commands
    send_init_sequence();
  }

  void update() {
    auto start = micros();
    select();

    command(); write8(0x2C); // memory write
    data();

    // more beats
    FLEXIO3_CTRL &= ~FLEXIO_CTRL_FLEXEN;
    FLEXIO3_TIMCMP0 = ((4 /*beats*/ * 2 - 1) << 8) | (FLEXIO3_TIMCMP0 & 0xFF);
    FLEXIO3_CTRL |= FLEXIO_CTRL_FLEXEN;

    if(cur_screen_mode == ScreenMode::lores){
      for(int y = 0; y < 240; y++) {
        auto ptr = screen_fb + (y / 2 * 160 * 3); // only increment every pther line

        for(int x = 0; x < 160; x++) {
          uint8_t r = *ptr++, g = *ptr++, b = *ptr++;
          uint16_t col0 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);

          FLEXIO3_SHIFTBUFBYS0 = col0 << 16 | col0; // horizontal double

          while(!(FLEXIO3_SHIFTSTAT & (1 << 0)));
        }
      }

    } else if(cur_screen_mode == ScreenMode::hires) {
      auto ptr = screen_fb;
      for(int y = 0; y < 240; y++) {
        for(int x = 0; x < 160; x++) {
          uint8_t r = *ptr++, g = *ptr++, b = *ptr++;
          uint16_t col0 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);

          r = *ptr++, g = *ptr++, b = *ptr++;
          uint16_t col1 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
          FLEXIO3_SHIFTBUFBYS0 = col0 << 16 | col1;

          while(!(FLEXIO3_SHIFTSTAT & (1 << 0)));
        }
      }
    }

    // back to normal
    FLEXIO3_CTRL &= ~FLEXIO_CTRL_FLEXEN;
    FLEXIO3_TIMCMP0 = ((1 /*beats*/ * 2 - 1) << 8) | (FLEXIO3_TIMCMP0 & 0xFF);
    FLEXIO3_CTRL |= FLEXIO_CTRL_FLEXEN;

    deselect();

    auto end = micros();
    Serial.printf("FT %ius\n", end - start);
  }

  bool set_screen_mode_format(ScreenMode mode, SurfaceTemplate &new_surf_template) {
    if(new_surf_template.format == (PixelFormat)-1)
      new_surf_template.format = PixelFormat::RGB;

    switch(mode) {
      case ScreenMode::lores:
        if(new_surf_template.bounds.empty())
          new_surf_template.bounds = lores_screen_size;
        else
          new_surf_template.bounds /= 2;
        break;

      case ScreenMode::hires:
      case ScreenMode::hires_palette:
        if(new_surf_template.bounds.empty())
          new_surf_template.bounds = hires_screen_size;
        break;
    }

    // support check
    if(new_surf_template.bounds != hires_screen_size && new_surf_template.bounds != lores_screen_size)
      return false;

    if(mode == ScreenMode::hires_palette)
      return false;

    if(new_surf_template.format != blit::PixelFormat::RGB)
      return false;

    // set data
    new_surf_template.data = screen_fb;

    cur_screen_mode = mode;

    return true;
  }
}
