set(BLIT_BOARD_NAME "Tufty 2350")

blit_driver(display dbi)
blit_driver(input "gpio;tca9555")
blit_driver(led pwm_mono)

if(TUFTY_BUTTONS_LEFT_RIGHT)
    list(APPEND BLIT_BOARD_DEFINITIONS TUFTY_BUTTONS_LEFT_RIGHT)
endif()
