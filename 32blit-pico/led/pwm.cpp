#include "led.hpp"

#include <cmath>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"

#include "config.h"

#include "engine/api_private.hpp"

static const int led_pins[]{LED_R_PIN, LED_G_PIN, LED_B_PIN};

void init_led() {
  pwm_config cfg = pwm_get_default_config();
#ifdef LED_INVERTED
  pwm_config_set_output_polarity(&cfg, true, true);
#endif

  for(auto &pin : led_pins) {
    pwm_set_wrap(pwm_gpio_to_slice_num(pin), 65535);
    pwm_init(pwm_gpio_to_slice_num(pin), &cfg, true);
    gpio_set_function(pin, GPIO_FUNC_PWM);
  }

  bi_decl(bi_1pin_with_name(led_pins[0], "Red LED"));
  bi_decl(bi_1pin_with_name(led_pins[1], "Green LED"));
  bi_decl(bi_1pin_with_name(led_pins[2], "Blue LED"));
}

void update_led() {
  using namespace blit;

  const float gamma = 2.8;
  uint16_t value = (uint16_t)(std::pow((float)(api_data.LED.r) / 255.0f, gamma) * 65535.0f + 0.5f);
  pwm_set_gpio_level(led_pins[0], value);
  value = (uint16_t)(std::pow((float)(api_data.LED.g) / 255.0f, gamma) * 65535.0f + 0.5f);
  pwm_set_gpio_level(led_pins[1], value);
  value = (uint16_t)(std::pow((float)(api_data.LED.b) / 255.0f, gamma) * 65535.0f + 0.5f);
  pwm_set_gpio_level(led_pins[2], value);
}