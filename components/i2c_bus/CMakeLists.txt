# Do NOT include() any ESP-IDF scripts here.
idf_component_register(
    SRCS "src/i2c_bus.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos
)
