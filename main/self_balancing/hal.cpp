
#include "hal.h"
#include "esp_log.h"
#include "lvgl.h"

#define TAG "HAL"

void HAL::Init()
{
    ESP_LOGI(TAG, "init mpu...");
    imu_init();
    ESP_LOGI(TAG, "init motor...");
    motor_init();
}
