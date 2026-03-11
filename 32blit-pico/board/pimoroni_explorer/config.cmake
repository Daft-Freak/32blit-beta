set(BLIT_BOARD_NAME "Explorer")

set(BLIT_BOARD_DEFINITIONS
    PICO_AUDIO_PWM_MONO_PIN=12
    PICO_AUDIO_PWM_PIO=1
    # bit tight on RAM in the loader
    PICO_AUDIO_BUFFER_SAMPLE_LENGTH=256 #576
)

blit_driver(audio pwm)
blit_driver(display dbi)
blit_driver(input "gpio;tca9555")
