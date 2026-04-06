idf_component_register(
    SRCS "main.c" "neopixel.c" "ds3231.c" "oled_display.c" "nvm_storage.c" "stepper.c" "buttons.c" "as5600.c" "calibration.c" "flipdot.c" "timekeeping.c" "animations.c" "network.c" "web_server.c" "action_log.c" "ota_update.c"
    INCLUDE_DIRS "."
    EMBED_FILES "${CMAKE_SOURCE_DIR}/frontend/index.html"
)
