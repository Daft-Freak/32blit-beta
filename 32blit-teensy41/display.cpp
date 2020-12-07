#include <cstdint>

#include "avr/pgmspace.h"
#include "core_pins.h"
#include "imxrt.h"
#include "pins_arduino.h"

#include "usb_serial.h"

#include "display.hpp"

using namespace blit;

namespace display {
  DMAMEM static uint8_t screen_fb[320 * 240 * 3]; // possibly EXTMEM

  static Surface lores_screen(screen_fb, PixelFormat::RGB, Size(160, 120));
  static Surface hires_screen(screen_fb, PixelFormat::RGB, Size(320, 240));
  static Surface hires_palette_screen(screen_fb, PixelFormat::P, Size(320, 240));

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

  static void write16(uint16_t v) {
    write8(v >> 8);
    write8(v & 0xFF);
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
    data();
    read_idle();

    digitalWriteFast(resetPin, 0);
    delay(1); // ?
    digitalWriteFast(resetPin, 1);

    delay(10);

    // begin
    select();

    // power control 1
    write8(0xC0);
    data(); write8(0x23); //4.6v, default 4.5v(0x21)

    // VCOM control 1  
    command(); write8(0xC5);
    data(); write16(0x2B2B); // 3.775v, -1.425v, default 3.925v(0x31), -1.0v (0x3C)

    // memory access control
    command(); write8(0x36);
    data(); write8(0xE0); // MV, MY, MX, BGR

    // pixel format set
    command(); write8(0x3A);
    data(); write8(0x55); // 16bpp
  
    // sleep out
    command(); write8(0x11);
    delay(120);

    // display on
    write8(0x29);

    // column address set
    write8(0x2A);
    data(); write16(0); write16(319);

    // page address set
    command(); write8(0x2B);
    data(); write16(0); write16(239);

    deselect();
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

  Surface &set_screen_mode(ScreenMode mode) {
    switch(mode) {
      case ScreenMode::lores:
        screen = lores_screen;
        break;

      case ScreenMode::hires:
        screen = hires_screen;
        break;

      case ScreenMode::hires_palette:
        screen = hires_palette_screen;
        break;
    }

    cur_screen_mode = mode;

    return screen;
  }
}