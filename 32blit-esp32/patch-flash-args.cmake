file(READ ${IN_FILE} FILE_CONTENT)

# replace the generator expression with a fixed target
string(REPLACE "$<TARGET_PROPERTY:__idf_build_target,EXECUTABLE_NAME>" "${TARGET_NAME}" FILE_CONTENT ${FILE_CONTENT})

# patch bootloader/partition table paths
string(REPLACE "bootloader/" "${BIN_DIR}/bootloader/" FILE_CONTENT ${FILE_CONTENT})
string(REPLACE "partition_table/" "${BIN_DIR}/partition_table/" FILE_CONTENT ${FILE_CONTENT})

file(WRITE ${OUT_FILE} ${FILE_CONTENT})
