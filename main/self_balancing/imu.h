#pragma once
#include "BMI270.h"
#include "filter.h"
#include "esp_err.h"
#include "esp_log.h"
#include "board.h"

class Imu {
public:
    Imu(i2c_master_bus_handle_t i2c_handle=nullptr,
        SemaphoreHandle_t i2c_semaphore=nullptr,
        FilterType type=FilterType::KALMAN,
        CoordinateSystem coord_sys = CoordinateSystem::X_FORWARD);

    esp_err_t init(i2c_master_bus_handle_t i2c_handle=nullptr);
    void update();

    float getPitch();
    float getYaw();
    float getRoll();

    float lowPassGyroPitch();
    float lowPassGyroZ();

private:
    BMI270 bmi270_;
    Filter filter_;
};