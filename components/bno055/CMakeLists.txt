# Do NOT include() any ESP-IDF scripts here.
idf_component_register(
    SRCS "bno055.c" "bno055_support.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos
)

# Map the Kconfig boolean to the macro expected by Bosch’s template
if(CONFIG_BNO055_API)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC BNO055_API=1)
endif()
