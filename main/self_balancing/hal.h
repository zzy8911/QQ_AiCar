#ifndef __HAL_H__
#define __HAL_H__
// #include "button_event.h"
#include "HAL_Def.h"
// #include "CommonMacro.h"
#include <esp_log.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_platform.h"

typedef enum {
    IMU_ENCODER_I2C_BUS = 0,
    ENCODER_I2C_BUS,
} I2C_BUS;

namespace HAL
{
    void Init();

    void  imu_init(void);
    void  imu_update(void *pvParameters);
    void  imu_update(void);
    float imu_get_abs_yaw(void);
    float imu_get_pitch(void);
    float imu_get_yaw(void);
    float imu_get_gyro_z(void);
    float lowPassGyroX(float alpha=0.2);
    float lowPassGyroZ(float alpha=0.2);

    i2c_master_bus_handle_t get_i2c_bus(I2C_BUS num);
    extern SemaphoreHandle_t i2c_mutex;
}
#endif