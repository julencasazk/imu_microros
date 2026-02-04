idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES bno055 i2c_bus micro_ros_transport micro_ros_espidf_component
)
