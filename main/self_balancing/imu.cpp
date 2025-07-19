#include "hal.h"
#include "HAL_Def.h"
#include "mpu6050.h"
#include "ICM42688.h"
#include <memory>

#define TAG "HAL_IMU"

static std::unique_ptr<ICM42688> imu = nullptr;

TaskHandle_t handleTaskIMU;
void HAL::imu_update(void *pvParameters)
{
    while (1) {
        imu_update();

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void HAL::imu_update(void)
{
    imu->getAGT();
    // ESP_LOGI(TAG, "%f\t%f\t%f\t%f\t%f\t%f\t%f",
    //                 imu->accX(), imu->accY(), imu->accZ(),
    //                 imu->gyrX(), imu->gyrY(), imu->gyrZ(), imu->temp());

    imu->filter();
    // ESP_LOGI(TAG, "%f", imu_get_pitch());
}

void HAL::imu_init(void)
{
    imu = std::make_unique<ICM42688>(HAL::get_i2c_bus(IMU_ENCODER_I2C_BUS), 0x68);
    int status = imu->begin();
	if (status < 0) {
        ESP_LOGE(TAG, "IMU initialization unsuccessful");
		ESP_LOGE(TAG, "Check IMU wiring or try cycling power");
		ESP_LOGE(TAG, "Status: %d", status);
		while (1) {}
    }

	// setting the accelerometer full scale range to +/-2G
	imu->setAccelFS(ICM42688::gpm2);
	// setting the gyroscope full scale range to +/-500 deg/s
	imu->setGyroFS(ICM42688::dps500);

	// set output data rate to 1k Hz
	imu->setAccelODR(ICM42688::odr1k);
	imu->setGyroODR(ICM42688::odr1k);

	imu->setFilter(HAL::KALMAN);

	// ESP_LOGI(TAG, "ax,ay,az,gx,gy,gz,temp_C");

    /* Move it to motor task, because as5600 sensor and imu sensor use the same I2C bus. */
    // esp_err_t ret = xTaskCreatePinnedToCore(
    //     imu_update,
    //     "IMUThread",
    //     4096,
    //     nullptr,
    //     2,
    //     &handleTaskIMU,
    //     1);
    // if (ret != pdPASS) {
    //     ESP_LOGE(TAG, "start imu_run task failed.");
    //     // return -1;
    // }
}

float HAL::imu_get_pitch(void)
{
    return imu->getPitch(); /* 0-180  -180 - 0 */
}

float HAL::lowPassGyroX(float alpha)
{
    return imu->lowPassGyroX(alpha);
}

float HAL::lowPassGyroZ(float alpha)
{
    return imu->lowPassGyroZ(alpha);
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
