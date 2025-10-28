# Do NOT include() any ESP-IDF scripts here.
idf_component_register(
    SRCS "src/bno055.c"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus driver
)
