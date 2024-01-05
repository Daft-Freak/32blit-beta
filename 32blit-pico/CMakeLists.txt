if(TARGET BlitHalPico)
    return()
endif()

# prevent find_package errors in pico_add_uf2_output later
set(PICO_SDK_VERSION_MAJOR ${PICO_SDK_VERSION_MAJOR} PARENT_SCOPE)
set(PICO_SDK_VERSION_MINOR ${PICO_SDK_VERSION_MINOR} PARENT_SCOPE)
set(PICO_SDK_VERSION_REVISION ${PICO_SDK_VERSION_REVISION} PARENT_SCOPE)

add_subdirectory(../32blit-shared ../32blit-shared)

add_library(BlitHalPico INTERFACE)
target_sources(BlitHalPico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/blit_launch.cpp
    ${CMAKE_CURRENT_LIST_DIR}/display.cpp
    ${CMAKE_CURRENT_LIST_DIR}/file.cpp
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/multiplayer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/overlay.cpp
    ${CMAKE_CURRENT_LIST_DIR}/sensor.cpp
    ${CMAKE_CURRENT_LIST_DIR}/usb.cpp
    ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
)

target_link_libraries(BlitHalPico INTERFACE
    hardware_dma hardware_i2c hardware_pio hardware_pwm hardware_spi
    pico_multicore pico_stdlib pico_unique_id pico_rand
    tinyusb_device
    FatFsBlitAPI
    LauncherShared
)

if(32BLIT_PICO_2350)
    target_sources(BlitHalPico INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/psram.c
    )
endif()

target_include_directories(BlitHalPico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR} # for tusb_config
)

target_compile_definitions(BlitHalPico INTERFACE
    PICO_AUDIO_DMA_IRQ=1
)

target_compile_options(BlitHalPico INTERFACE
    -Wno-ignored-qualifiers # pico-sdk generates a lot of these
)

# generate PIO headers (has to be after SDK init)
pico_generate_pio_header(BlitHalPico ${CMAKE_CURRENT_LIST_DIR}/dbi-spi.pio)
pico_generate_pio_header(BlitHalPico ${CMAKE_CURRENT_LIST_DIR}/dbi-8bit.pio)
pico_generate_pio_header(BlitHalPico ${CMAKE_CURRENT_LIST_DIR}/audio/i2s.pio)
pico_generate_pio_header(BlitHalPico ${CMAKE_CURRENT_LIST_DIR}/display/dpi.pio)
pico_generate_pio_header(BlitHalPico ${CMAKE_CURRENT_LIST_DIR}/spi.pio)
pico_generate_pio_header(BlitHalPico ${CMAKE_CURRENT_LIST_DIR}/sd_card.pio)
pico_generate_pio_header(BlitHalPico ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# include picovision drivers
if(BLIT_DISPLAY_DRIVER STREQUAL "picovision")
    include(picovision_import.cmake)
elseif(BLIT_DISPLAY_DRIVER STREQUAL "gbacart")
    include(FetchContent)

    FetchContent_Populate(gbacart
        GIT_REPOSITORY https://github.com/Daft-Freak/pico-gba-cart
        GIT_TAG 0f61d075e3898573c5cbc60aacaf95b664677a71
    )

    add_subdirectory(${gbacart_SOURCE_DIR}/lib ${gbacart_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# driver sources
list(TRANSFORM BLIT_SENSOR_DRIVER PREPEND ${CMAKE_CURRENT_LIST_DIR}/sensor/ OUTPUT_VARIABLE SENSOR_SOURCES)
list(TRANSFORM SENSOR_SOURCES APPEND .cpp)

target_sources(BlitHalPico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/audio/${BLIT_AUDIO_DRIVER}.cpp
    ${CMAKE_CURRENT_LIST_DIR}/display/${BLIT_DISPLAY_DRIVER}.cpp
    ${CMAKE_CURRENT_LIST_DIR}/input/${BLIT_INPUT_DRIVER}.cpp
    ${CMAKE_CURRENT_LIST_DIR}/led/${BLIT_LED_DRIVER}.cpp
    ${SENSOR_SOURCES}
    ${CMAKE_CURRENT_LIST_DIR}/storage/${BLIT_STORAGE_DRIVER}.cpp
    ${CMAKE_CURRENT_LIST_DIR}/usb/${BLIT_USB_DRIVER}.cpp
)

# BLIT_SENSOR_XXX=1 defines
list(TRANSFORM BLIT_SENSOR_DRIVER TOUPPER OUTPUT_VARIABLE SENSOR_DEFINES)
list(TRANSFORM SENSOR_DEFINES PREPEND BLIT_SENSOR_)
target_compile_definitions(BlitHalPico INTERFACE ${SENSOR_DEFINES})

if(BLIT_ENABLE_CORE1)
    list(APPEND BLIT_BOARD_DEFINITIONS ENABLE_CORE1)
endif()

target_compile_definitions(BlitHalPico INTERFACE ${BLIT_BOARD_DEFINITIONS})
target_link_libraries(BlitHalPico INTERFACE ${BLIT_BOARD_LIBRARIES})

set(BLIT_BOARD_DEFINITIONS ${BLIT_BOARD_DEFINITIONS} PARENT_SCOPE)

# for enabling stdio_usb
set(BLIT_USB_DRIVER ${BLIT_USB_DRIVER} PARENT_SCOPE)
