#include "board.h"
#include "BMI270.h"
#include <memory>
#include "filter.h"
#include "imu.h"

#define TAG "IMU"

Imu::Imu(i2c_master_bus_handle_t i2c_handle,
         SemaphoreHandle_t i2c_semaphore,
         FilterType type, CoordinateSystem coord_sys)
    : bmi270_(i2c_handle, i2c_semaphore, 0x68)
    , filter_(type, coord_sys)
{
    // 构造仅初始化成员，不做设备/滤波器初始化（放到 init()）
}

esp_err_t Imu::init(i2c_master_bus_handle_t i2c_handle)
{
    // 初始化底层 BMI270
    esp_err_t ret = bmi270_.init(i2c_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 begin() failed: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Imu::init OK");
    return ESP_OK;
}

void Imu::update()
{
    esp_err_t r = bmi270_.update();
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "BMI270 update failed: %d", r);
        return;
    }

    filter_.update(bmi270_.accX(), bmi270_.accY(), bmi270_.accZ(),
                   bmi270_.gyrX(), bmi270_.gyrY(), bmi270_.gyrZ());
}

float Imu::getPitch()
{
    return filter_.getPitch();
}

float Imu::getRoll()
{
    return filter_.getRoll();
}

float Imu::getYaw()
{
    return filter_.getYaw();
}

float Imu::lowPassGyroPitch()
{
    return filter_.lowPassGyroPitch();
}

float Imu::lowPassGyroZ()
{
    return filter_.lowPassGyroZ();
}