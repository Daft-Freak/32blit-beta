set(BLIT_BOARD_NAME "Adafruit Fruit Jam")

blit_driver(audio i2s) # TODO: configure DAC
blit_driver(display hstx_dv)
blit_driver(input usb_hid)
blit_driver(storage sd_spi)
blit_driver(usb host) # TODO: reconfigure for PIO USB
