#include "hal.h"
#include "HAL_Def.h"
#include "mpu6050.h"

#define TAG "HAL_IMU"

static mpu6050_handle_t mpu;
static complimentary_angle_t angle;

TaskHandle_t handleTaskIMU;
void HAL::imu_update(void *pvParameters)
{
    mpu6050_acce_value_t acc;
    mpu6050_gyro_value_t gyro;

    while (1) {
        mpu6050_get_acce(mpu, &acc);
        mpu6050_get_gyro(mpu, &gyro);
        // minus offset
        gyro.gyro_x -= ((mpu6050_dev_t *)mpu)->gyroXoffset;
        gyro.gyro_y -= ((mpu6050_dev_t *)mpu)->gyroYoffset;
        gyro.gyro_z -= ((mpu6050_dev_t *)mpu)->gyroZoffset;
        mpu6050_complimentory_filter(mpu, &acc, &gyro, &angle);

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // remember to delete the mpu6050 object
    mpu6050_delete(mpu);
}

void HAL::imu_init(void)
{
    // 创建 MPU6050 对象
    mpu = mpu6050_create(get_i2c_bus(IMU_BUS), MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu, ACCE_FS_2G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu);

    mpu6050_calc_gyro_offsets(mpu, true, 1000, 3000);

    ESP_LOGD(TAG, "imu init pitch offset : %0.2f\n", imu_get_pitch());

    esp_err_t ret = xTaskCreatePinnedToCore(
        imu_update,
        "IMUThread",
        4096,
        nullptr,
        2,
        &handleTaskIMU,
        1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "start imu_run task failed.");
        // return -1;
    }
}

float HAL::imu_get_pitch(void)
{
    return angle.pitch; /* 0-180  -180 - 0 */
}

float HAL::imu_get_yaw(void)
{
    // FIXME: 需要修改
    return 0;
}

/*
 * getGyroY = pitch
 * getGyroZ = yaw
 * 
 */
float HAL::imu_get_gyro_z(void)
{
    // FIXME: 需要修改
    return 0;
}

float HAL::imu_get_abs_yaw(void)
{
    // FIXME: 需要修改
    return 0;
}
