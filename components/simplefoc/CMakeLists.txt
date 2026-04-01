file(GLOB_RECURSE SOURCES 
    "*.cpp"
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS
        .
        common
        current_sense
        port
        sensors
    REQUIRES driver esp_timer esp_adc espressif__i2c_bus
)
