set(BLIT_BOARD_NAME "VGA Board")

set(BLIT_BOARD_DEFINITIONS
    # these are duplicated so we can build for non-pico boards (vgaboard.h includes pico.h)
    PICO_SCANVIDEO_COLOR_PIN_BASE=0
    PICO_SCANVIDEO_SYNC_PIN_BASE=16
)

blit_driver(audio i2s)
blit_driver(display dpi)
blit_driver(input usb_hid)
blit_driver(storage sd_spi)
blit_driver(usb host)

set(BLIT_ENABLE_CORE1 TRUE)
