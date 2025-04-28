# ESP-IDF setup
include($ENV{IDF_PATH}/tools/cmake/idf.cmake)
# ESP_TARGET is how we detect an ESP32 build, so should be set
idf_build_process(${ESP_TARGET})

# grab some internal vars for later
# (components/esptool_py/project_include.cmake)
set(ELF2IMAGE_ARGS ${esptool_elf2image_args} PARENT_SCOPE)
set(ESPTOOLPY ${ESPTOOLPY} PARENT_SCOPE)

# prepare to generate extra flash args
# (esptool_py_flash_target)
set(flash_args_content "$<JOIN:$<TARGET_PROPERTY:flash,SUB_ARGS>, >\n$<JOIN:$<TARGET_PROPERTY:flash,IMAGES>,\n>")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/template_args.in" CONTENT "${flash_args_content}")

set(FLASH_ARGS_TEMPLATE ${CMAKE_CURRENT_BINARY_DIR}/template_args.in)
set(FLASH_ARGS_PATCH_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/patch-flash-args.cmake)
set(FLASH_ARGS_TEMPLATE ${FLASH_ARGS_TEMPLATE} PARENT_SCOPE)
set(FLASH_ARGS_PATCH_SCRIPT ${FLASH_ARGS_PATCH_SCRIPT} PARENT_SCOPE)

# "HAL" library
add_library(BlitHalESP32 INTERFACE)

target_sources(BlitHalESP32 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
)

target_link_libraries(BlitHalESP32 INTERFACE
    idf::newlib
)

# we need to build something here, and it needs to link to an idf:: library or we get an error of
# "The SOURCES of "__idf_riscv" use a generator expression that depends on the SOURCES themselves."
# for some reason (there aren't any generator expressions in SOURCES)
add_executable(i-am-a-workaround stub.c)
target_link_libraries(i-am-a-workaround idf::newlib)
idf_build_executable(i-am-a-workaround)

# executable helpers
function(blit_executable NAME)
    message(STATUS "Processing ${NAME}")
    blit_executable_args(${ARGN})

    add_executable(${NAME} ${SOURCES})
    target_link_libraries(${NAME} BlitEngine BlitHalESP32)

    # this only works properly for one executable
    # (binary generation and flashing will only work for the last one)
    idf_build_executable(${NAME})

    # generate bins for everything
    add_custom_command(TARGET ${NAME} POST_BUILD
        COMMENT "Building ${NAME}.bin"
        COMMAND ${ESPTOOLPY} elf2image ${esptool_elf2image_args} -o ${NAME}.bin $<TARGET_FILE:${NAME}>
        VERBATIM
    )

    # patch together flash args
    add_custom_command(TARGET ${NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} "-DIN_FILE=${FLASH_ARGS_TEMPLATE}" "-DOUT_FILE=${CMAKE_CURRENT_BINARY_DIR}/${NAME}_args" "-DTARGET_NAME=${NAME}" "-DBIN_DIR=${CMAKE_BINARY_DIR}" -P ${FLASH_ARGS_PATCH_SCRIPT}
        VERBATIM
    )

    # flash target
    if(FLASH_PORT)
        add_custom_target(${NAME}.flash DEPENDS ${NAME} COMMAND ${ESPTOOLPY} -p ${FLASH_PORT} write_flash "@${NAME}_args")
    endif()
endfunction()

function(blit_metadata TARGET FILE)
    if(NOT IS_ABSOLUTE ${FILE})
        set(FILE ${CMAKE_CURRENT_SOURCE_DIR}/${FILE})
    endif()

    # stub
endfunction()
