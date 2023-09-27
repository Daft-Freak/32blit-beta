# make sure engine is build with PIC
set_target_properties(BlitEngine PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(BlitHalLibRetro STATIC
	api.cpp
	default_metadata.cpp
	main.cpp
)

target_link_libraries(BlitHalLibRetro PUBLIC BlitEngine)
set_target_properties(BlitHalLibRetro PROPERTIES POSITION_INDEPENDENT_CODE ON)

function(blit_executable NAME)
	message(STATUS "Processing ${NAME}")
	blit_executable_args(${ARGN})

	add_library(${NAME} SHARED ${SOURCES})

	install(TARGETS ${NAME}
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
		BUNDLE DESTINATION bin
	)

	if (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
		target_link_libraries(${NAME} BlitHalLibRetro)
	elseif(${CMAKE_SYSTEM_NAME} STREQUAL Darwin)
  		target_link_libraries(${NAME} -Wl,-force_load BlitHalLibRetro)
	else()
  		target_link_libraries(${NAME} -Wl,--whole-archive BlitHalLibRetro -Wl,--no-whole-archive)
	endif()

	set_target_properties(${NAME} PROPERTIES OUTPUT_NAME ${NAME}_libretro PREFIX "")
endfunction()

function(blit_metadata TARGET FILE)
	if(NOT EXISTS ${FILE})
		set(FILE ${CMAKE_CURRENT_SOURCE_DIR}/${FILE})
	endif()

	# cause cmake to reconfigure whenever the asset list changes
	set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${FILE})

	# parse the metadata to variables
	execute_process(
		COMMAND ${32BLIT_TOOLS_EXECUTABLE} cmake --config ${FILE} --cmake ${CMAKE_CURRENT_BINARY_DIR}/metadata.cmake
		RESULT_VARIABLE TOOL_RESULT
	)
	if(${TOOL_RESULT})
		message(FATAL_ERROR "Reading metadata config failed!\n")
	endif()

	include(${CMAKE_CURRENT_BINARY_DIR}/metadata.cmake)

	if(APPLE)
		set_target_properties(${TARGET} PROPERTIES
			OUTPUT_NAME ${METADATA_TITLE}
			MACOSX_BUNDLE_BUNDLE_NAME ${METADATA_TITLE}
			MACOSX_BUNDLE_SHORT_VERSION_STRING ${METADATA_VERSION}
			MACOSX_BUNDLE_LONG_VERSION_STRING ${METADATA_VERSION}
			MACOSX_BUNDLE_COPYRIGHT "${METADATA_AUTHOR}"
			MACOSX_BUNDLE_ICON_FILE icon
		)

		set(ICON ${CMAKE_CURRENT_BINARY_DIR}/icon.icns)

		add_custom_command(
			OUTPUT ${ICON}
			COMMAND cd ${CMAKE_CURRENT_SOURCE_DIR} && ${32BLIT_TOOLS_EXECUTABLE} metadata --force --config ${FILE} --icns ${ICON}
			DEPENDS ${METADATA_DEPENDS} ${FILE}
			VERBATIM
		)

		target_sources(${TARGET} PRIVATE ${ICON})
		set_source_files_properties(${ICON} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
	endif()

	string(CONFIGURE "
const char *metadata_title = \"\${METADATA_TITLE}\";
const char *metadata_author = \"\${METADATA_AUTHOR}\";
const char *metadata_description = \"\${METADATA_DESCRIPTION}\";
const char *metadata_version = \"\${METADATA_VERSION}\";
const char *metadata_url = \"\${METADATA_URL}\";
const char *metadata_category = \"\${METADATA_CATEGORY}\";"
		METADATA_CONTENT ESCAPE_QUOTES
	)

	file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_metadata.cpp CONTENT "${METADATA_CONTENT}")

	target_sources(${TARGET} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_metadata.cpp)
endfunction()
