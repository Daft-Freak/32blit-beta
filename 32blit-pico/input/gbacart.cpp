#include "input.hpp"

#include "engine/api_private.hpp"
#include "engine/input.hpp"

#include "gbacart.h"

void init_input() {
}

void update_input() {
  using namespace blit;

  auto cart_api = gbacart_get_api();
  auto button_state = ~cart_api->buttons;

  api_data.buttons = ((button_state & (1 << 5)) ? uint32_t(Button::DPAD_LEFT) : 0)
                   | ((button_state & (1 << 4)) ? uint32_t(Button::DPAD_RIGHT) : 0)
                   | ((button_state & (1 << 6)) ? uint32_t(Button::DPAD_UP) : 0)
                   | ((button_state & (1 << 7)) ? uint32_t(Button::DPAD_DOWN) : 0)
                   | ((button_state & (1 << 0)) ? uint32_t(Button::A) : 0)
                   | ((button_state & (1 << 1)) ? uint32_t(Button::B) : 0)
                   | ((button_state & (1 << 9)) ? uint32_t(Button::X) : 0)  // L
                   | ((button_state & (1 << 8)) ? uint32_t(Button::Y) : 0)  // R 
                   | ((button_state & (1 << 2)) ? uint32_t(Button::MENU) : 0)  // Select
                   | ((button_state & (1 << 3)) ? uint32_t(Button::HOME) : 0); // Start
}
