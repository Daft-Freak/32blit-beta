set(TEENSY_CORE_PATH "cores/teensy4" CACHE PATH "Path to Teensy 4 core")
set(TEENSY_TOOLS_PATH "${TEENSY_CORE_PATH}/../../../../tools" CACHE PATH "Path to Teensy tools")

list(APPEND SOURCES
	${TEENSY_CORE_PATH}/analog.c
	${TEENSY_CORE_PATH}/bootdata.c
	${TEENSY_CORE_PATH}/clockspeed.c
	${TEENSY_CORE_PATH}/digital.c
	${TEENSY_CORE_PATH}/EventResponder.cpp
	${TEENSY_CORE_PATH}/delay.c
	${TEENSY_CORE_PATH}/HardwareSerial.cpp
	${TEENSY_CORE_PATH}/nonstd.c
	${TEENSY_CORE_PATH}/Print.cpp
	${TEENSY_CORE_PATH}/pwm.c
	${TEENSY_CORE_PATH}/serialEvent.cpp
	${TEENSY_CORE_PATH}/startup.c
	${TEENSY_CORE_PATH}/tempmon.c
	${TEENSY_CORE_PATH}/usb.c
	${TEENSY_CORE_PATH}/usb_desc.c
	${TEENSY_CORE_PATH}/usb_inst.cpp
	${TEENSY_CORE_PATH}/usb_serial.c
	${TEENSY_CORE_PATH}/yield.cpp

	display.cpp
	main.cpp
)

add_library(BlitHalTeensy41 OBJECT ${SOURCES})

set(INCLUDE_DIRS
	${TEENSY_CORE_PATH}
	${CMAKE_CURRENT_SOURCE_DIR}/../32blit
)
#set(HAL_INCLUDE_DIRS ${INCLUDE_DIRS} PARENT_SCOPE)

set(DEFINITIONS
	-DF_CPU=600000000
	-DUSB_SERIAL
	-DLAYOUT_US_ENGLISH
	-DUSING_MAKEFILE
	-D__IMXRT1062__
	-DARDUINO_TEENSY41
)
#set(HAL_DEFINITIONS ${DEFINITIONS} PARENT_SCOPE)

target_include_directories(BlitHalTeensy41
	PRIVATE
		${INCLUDE_DIRS}
)

target_compile_definitions(BlitHalTeensy41
	PRIVATE
		${DEFINITIONS}
)

#target_compile_options(BlitHalTeensy41 PUBLIC "$<$<CONFIG:RELEASE>:-Os>")
set_target_properties(BlitHalTeensy41 PROPERTIES
	#INTERPROCEDURAL_OPTIMIZATION TRUE
	CXX_EXTENSIONS TRUE
)

function(blit_executable_int_flash NAME SOURCES)
	add_executable(${NAME} $<TARGET_OBJECTS:BlitHalTeensy41> ${SOURCES} ${ARGN})

	install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.bin ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.dfu
		DESTINATION bin
	)

	# -specs=nano.specs 
	set_target_properties(${NAME} PROPERTIES LINK_FLAGS "-T ${MCU_LINKER_SCRIPT} ${MCU_LINKER_FLAGS_INT}")
	set_target_properties(${NAME} PROPERTIES LINK_DEPENDS ${MCU_LINKER_SCRIPT} SUFFIX ".elf")

	blit_executable_common(${NAME})
	#target_link_libraries(${NAME} ${NOPIC_STDLIBS})

	#target_include_directories(${NAME} PRIVATE ${HAL_INCLUDE_DIRS})
	#target_compile_definitions(${NAME} PRIVATE ${HAL_DEFINITIONS})
	#target_compile_options(${NAME} PUBLIC "$<$<CONFIG:RELEASE>:-Os>") # always optimise for size if targetting internal flash
endfunction()
