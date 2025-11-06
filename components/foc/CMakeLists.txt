idf_component_register(
    SRCS 
        "esp_hal_bldc_3pwm.cpp"
        "esp_hal_gpio.cpp"
        "esp_hal_misc.cpp"
        "inline_current.cpp"
        "lowpass_filter.cpp"
        "motor.cpp"
        "mt6701.cpp"
        "pid.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_adc espressif__i2c_bus
)
