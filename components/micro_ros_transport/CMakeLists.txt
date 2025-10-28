cmake_minimum_required(VERSION 3.5)

# Do NOT include($ENV{IDF_PATH}/.../project.cmake) or any idf.cmake here.
idf_component_register(
    SRCS "src/esp32_serial_transport.c"
    INCLUDE_DIRS "include"
    REQUIRES micro_ros_espidf_component driver freertos esp_timer
)
