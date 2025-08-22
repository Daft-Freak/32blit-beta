# driver helper
# can override driver choice by pre-setting BLIT_x_DRIVER
function(blit_driver DRV NAME)
    set(var BLIT_${DRV}_DRIVER)
    string(TOUPPER ${var} var)

    if(NOT ${var})
        set(${var} ${NAME} PARENT_SCOPE)
    endif()
endfunction()

if(NOT DEFINED ESP_BOARD)
  set(ESP_BOARD default)
  message(WARNING "Using default board...")
endif()

set(CONFIG_PATH ${CMAKE_CURRENT_LIST_DIR}/board/${ESP_BOARD})
set(BOARD_ID ${ESP_BOARD})

if(NOT EXISTS ${CONFIG_PATH}/config.cmake)
    set(CONFIG_PATH ${CMAKE_CURRENT_LIST_DIR}/board/pico)
    message(WARNING "Using default config for \"${ESP_BOARD}\"...")
endif()

include(${CONFIG_PATH}/config.cmake)
message("Using board config \"${BLIT_BOARD_NAME}\"")

if(EXISTS ${CONFIG_PATH}/config.h)
    list(APPEND BLIT_BOARD_DEFINITIONS "BLIT_BOARD_CONFIG=\"${CONFIG_PATH}/config.h\"")
endif()

# board id definition
string(TOUPPER ${BOARD_ID} BOARD_ID)
list(APPEND BLIT_BOARD_DEFINITIONS BLIT_BOARD_${BOARD_ID})

# default drivers
if(NOT BLIT_AUDIO_DRIVER)
    set(BLIT_AUDIO_DRIVER "none")
endif()
if(NOT BLIT_DISPLAY_DRIVER)
    set(BLIT_DISPLAY_DRIVER "none")
endif()
if(NOT BLIT_INPUT_DRIVER)
    set(BLIT_INPUT_DRIVER "none")
endif()

# driver dependencies
