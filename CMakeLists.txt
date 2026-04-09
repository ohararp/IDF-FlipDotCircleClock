cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "0.1.4")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(FlipDotCircleClock)

# Produce a versioned copy of the firmware binary after the image is generated.
# We keep the original FlipDotCircleClock.bin in place because `idf.py flash`
# (and the bootloader's flash_args manifest) reference that exact filename —
# renaming/removing it would break USB flashing. The versioned copy is what
# you upload through the web OTA card.
add_custom_target(versioned_bin ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}_v${PROJECT_VER}.bin
    COMMENT "Creating versioned binary: ${CMAKE_PROJECT_NAME}_v${PROJECT_VER}.bin"
    DEPENDS gen_project_binary
)
