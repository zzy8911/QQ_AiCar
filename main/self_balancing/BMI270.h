#pragma once
#include <stdint.h>
#include <stdio.h>
#include "bmi270.h"
#include "bmi270_interface.h"
#include <driver/i2c_master.h>
#include "esp_err.h"
#include "esp_log.h"
#include "filter.h"

class BMI270 {
public:
    BMI270(i2c_master_bus_handle_t i2c_handle, SemaphoreHandle_t i2c_semaphore=NULL, uint8_t address=0x68);
    int init();
    int update();

    float accX() const { return acc_x_; }
    float accY() const { return acc_y_; }
    float accZ() const { return acc_z_; }
    float gyrX() const { return gyr_x_; }
    float gyrY() const { return gyr_y_; }
    float gyrZ() const { return gyr_z_; }

private:
    int8_t setGyroConfig(uint8_t odr, uint8_t range);
    int8_t setAccelConfig(uint8_t odr, uint8_t range);
    float lsbToDps(int16_t val, float dps, uint8_t bit_width);
    float lsbToMps2(int16_t val, float g_range, uint8_t bit_width);

private:
    struct bmi2_dev dev_{};
    struct bmi2_sens_data sensor_data_{};
    i2c_master_bus_handle_t i2c_handle_;
    SemaphoreHandle_t i2c_semaphore_;
    uint8_t address_;

    float acc_x_{0}, acc_y_{0}, acc_z_{0};
    float gyr_x_{0}, gyr_y_{0}, gyr_z_{0};

    Filter *filter_ = nullptr;
};
