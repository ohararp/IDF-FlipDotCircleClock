cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "0.1.1")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(FlipDotCircleClock)

# Copy .bin with version in filename after binary image is generated
add_custom_target(versioned_bin ALL
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}_v${PROJECT_VER}.bin
    COMMENT "Creating versioned binary: ${CMAKE_PROJECT_NAME}_v${PROJECT_VER}.bin"
    DEPENDS gen_project_binary
)
