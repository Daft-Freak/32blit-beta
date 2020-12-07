#include <cstdint>

#include "core_pins.h"
#include "usb_serial.h"

#include "engine/engine.hpp"
#include "engine/api_private.hpp"

#include "display.hpp"

using namespace blit;

void init();

void render(uint32_t);
void update(uint32_t);

int main() {
  Serial.begin(9600); //dbg

  display::init();

  // api setup
  api.set_screen_mode = display::set_screen_mode;
  api.now = millis;

  ::set_screen_mode(ScreenMode::lores);

  blit::render = ::render;
  blit::update = ::update;

  // user init
  ::init();

  uint32_t last_render = 0;

  while(true) {
    tick(millis());

    auto now = millis();

    if(now - last_render >= 20) {
      ::render(now);
      display::update();
      last_render = now;
    }
  }
  
  return 0;
}