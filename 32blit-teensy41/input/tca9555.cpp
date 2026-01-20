#include <cstdio>

#include "core_pins.h"
#include "imxrt.h"
#include "pins_arduino.h"

#include "input.hpp"

// #include "config.h"

#include "engine/api_private.hpp"
#include "engine/input.hpp"


#ifndef TCA9555_ADDR
#define TCA9555_ADDR 0x21
#endif

// QwST Pad
#define TCA9555_LEFT_IO   2
#define TCA9555_RIGHT_IO  3
#define TCA9555_UP_IO     1
#define TCA9555_DOWN_IO   4
#define TCA9555_A_IO      14
#define TCA9555_B_IO      12
#define TCA9555_X_IO      15
#define TCA9555_Y_IO      13
#define TCA9555_START_IO  11
#define TCA9555_SELECT_IO 5

// I2C helpers
// TODO: put somewhere else
static void lpi2c4_init() {
   // I2C4 IO
  auto pad_config = IOMUXC_PAD_HYS      // hysteresis enabled
                  | IOMUXC_PAD_PUS(3)   // 22K up
                  | IOMUXC_PAD_PUE      // pull
                  | IOMUXC_PAD_PKE      // enable pull
                  | IOMUXC_PAD_ODE      // open-drain
                  | IOMUXC_PAD_SPEED(0) // low speed
                  | IOMUXC_PAD_DSE(4);  // drive strength?

  *portConfigRegister(24)  = 0x10/*SION*/ | 0; // SCL (ALT0)
  *portControlRegister(24) = pad_config;
  IOMUXC_LPI2C4_SCL_SELECT_INPUT = 1;

  *portConfigRegister(25)  = 0x10/*SION*/ | 0; // SDA (ALT0)
  *portControlRegister(25) = pad_config;
  IOMUXC_LPI2C4_SDA_SELECT_INPUT = 1;

  // I2C4 clocks (default base is 60MHz?)
  CCM_CCGR6 |= CCM_CCGR6_LPI2C4_SERIAL(CCM_CCGR_ON);

  // reset
  LPI2C4_MCR = LPI2C_MCR_RRF | LPI2C_MCR_RTF | LPI2C_MCR_RST;
  LPI2C4_MCR = 0;

  // most timing params taken from manual example for 400kHz
  LPI2C4_MCFGR0 = 0; // defaults
  LPI2C4_MCFGR1 = LPI2C_MCFGR1_PRESCALE(1); // /2
  LPI2C4_MCFGR2 = LPI2C_MCFGR2_FILTSDA(2) | LPI2C_MCFGR2_FILTSCL(2) | LPI2C_MCFGR2_BUSIDLE(100/*us*/ * 30);
  LPI2C4_MCFGR3 = LPI2C_MCFGR3_PINLOW((1000/*us*/ * 30) / 256); // set clock stretch timeout to ~1ms?

  LPI2C4_MCCR0 = LPI2C_MCCR0_DATAVD(8) | LPI2C_MCCR0_SETHOLD(17) | LPI2C_MCCR0_CLKHI(31) | LPI2C_MCCR0_CLKLO(40);
  // MCCR1 is used for high speed

  LPI2C4_MFCR = LPI2C_MFCR_TXWATER(1);

  LPI2C4_MCR = LPI2C_MCR_MEN; // enable
}

static void lpi2c4_write(uint8_t addr, const uint8_t *data, size_t len) {
  // wait until bus is not busy
  while(LPI2C4_MSR & LPI2C_MSR_BBF);

  // clear status
  LPI2C4_MSR = LPI2C_MSR_PLTF | LPI2C_MSR_FEF | LPI2C_MSR_ALF | LPI2C_MSR_NDF | LPI2C_MSR_SDF | LPI2C_MSR_EPF;

  LPI2C4_MTDR = LPI2C_MTDR_CMD_START | addr << 1 | 0; // start+addr

  for(size_t i = 0; i < len; i++) {
    LPI2C4_MSR = LPI2C_MSR_TDF; // clear
    while(!(LPI2C4_MSR & LPI2C_MSR_TDF)); // wait for fifo space
    LPI2C4_MTDR = data[i];
  }

  LPI2C4_MTDR = LPI2C_MTDR_CMD_STOP;

  // wait until STOP sent
  while(!(LPI2C4_MSR & LPI2C_MSR_SDF));
}

static void lpi2c4_read(uint8_t addr, uint8_t *data, size_t len) {
  // wait until bus is not busy
  while(LPI2C4_MSR & LPI2C_MSR_BBF);

  // clear status
  LPI2C4_MSR = LPI2C_MSR_PLTF | LPI2C_MSR_FEF | LPI2C_MSR_ALF | LPI2C_MSR_NDF | LPI2C_MSR_SDF | LPI2C_MSR_EPF;

  LPI2C4_MTDR = LPI2C_MTDR_CMD_START | addr << 1 | 1; // start+addr

  // FIXME: len > 256?
  LPI2C4_MTDR = LPI2C_MTDR_CMD_RECEIVE | ((len - 1) & 0xFF);

  bool err = false;

  for(size_t i = 0; i < len && !err; i++) {
    uint32_t rx_data;
    // keep reading until fifo not empty
    do {
      rx_data = LPI2C4_MRDR;
      auto status = LPI2C4_MSR;

      // FIFO error
      if(status & LPI2C_MSR_FEF) {
        // give up
        LPI2C4_MCR = LPI2C_MCR_RRF | LPI2C_MCR_RTF; // reset FIFO
        return;
      }

      // lost the bus
      if(status & LPI2C_MSR_ALF) {
        // give up
        LPI2C4_MCR = LPI2C_MCR_RRF | LPI2C_MCR_RTF; // reset FIFO
        return;
      }

      // NACK
      if(status & LPI2C_MSR_NDF) {
        // give up, but still try to stop
        LPI2C4_MCR = LPI2C_MCR_RRF | LPI2C_MCR_RTF; // reset FIFO
        err = true;
        break;
      }

    } while(rx_data & LPI2C_MRDR_RXEMPTY);
    data[i] = rx_data & 0xFF;
  }

  LPI2C4_MTDR = LPI2C_MTDR_CMD_STOP;

  // wait until STOP sent
  while(!(LPI2C4_MSR & LPI2C_MSR_SDF));
}

namespace input {
  static uint32_t last_read_time = 0;

  void init() {
    lpi2c4_init();

    // setup for reading
    uint8_t port = 0;
    lpi2c4_write(TCA9555_ADDR, &port, 1);
  }

  void update() {
    // limit update rate
    auto now = millis();

    if(now - last_read_time < 5)
      return;

    last_read_time = now;

    // read
    uint16_t gpio = 0;

    lpi2c4_read(TCA9555_ADDR, (uint8_t *)&gpio, 2);

    uint32_t new_buttons = 0;

    if(!(gpio & (1 << TCA9555_LEFT_IO)))
      new_buttons |= blit::Button::DPAD_LEFT;

    if(!(gpio & (1 << TCA9555_RIGHT_IO)))
      new_buttons |= blit::Button::DPAD_RIGHT;

    if(!(gpio & (1 << TCA9555_UP_IO)))
      new_buttons |= blit::Button::DPAD_UP;

    if(!(gpio & (1 << TCA9555_DOWN_IO)))
      new_buttons |= blit::Button::DPAD_DOWN;

    if(!(gpio & (1 << TCA9555_A_IO)))
      new_buttons |= blit::Button::A;

    if(!(gpio & (1 << TCA9555_B_IO)))
      new_buttons |= blit::Button::B;

    if(!(gpio & (1 << TCA9555_X_IO)))
      new_buttons |= blit::Button::X;

    if(!(gpio & (1 << TCA9555_Y_IO)))
      new_buttons |= blit::Button::Y;

    if(!(gpio & (1 << TCA9555_START_IO)))
      new_buttons |= blit::Button::HOME;

    if(!(gpio & (1 << TCA9555_SELECT_IO)))
      new_buttons |= blit::Button::MENU;

    blit::api_data.buttons = new_buttons;
  }
}
