
#include "hal.h"
#include "boards/self-balancing-robot/config.h"

#define TAG "HAL"

static i2c_master_bus_handle_t imu_encoder_i2c_bus_; // 42, 41, icm42688, as5600
static i2c_master_bus_handle_t encoder_i2c_bus_; // 1, 2, as5600

SemaphoreHandle_t HAL::i2c_mutex = nullptr;

static void init_i2c() {
    HAL::i2c_mutex = xSemaphoreCreateMutex();

    // Initialize I2C peripheral
    i2c_master_bus_config_t i2c0_bus_cfg = {
        .i2c_port = (i2c_port_t)I2C_NUM_0,
        .sda_io_num = IMU_ENCODER_SDA,
        .scl_io_num = IMU_ENCODER_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c0_bus_cfg, &imu_encoder_i2c_bus_));

    // Initialize I2C peripheral
    i2c_master_bus_config_t i2c1_bus_cfg = {
        .i2c_port = (i2c_port_t)I2C_NUM_1,
        .sda_io_num = ENCODER_SDA,
        .scl_io_num = ENCODER_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c1_bus_cfg, &encoder_i2c_bus_));

    ESP_LOGI(TAG, "init_i2c finished...");
}

i2c_master_bus_handle_t HAL::get_i2c_bus(I2C_BUS num)
{
    if (num == IMU_ENCODER_I2C_BUS) {
        if (imu_encoder_i2c_bus_ == nullptr) {
            ESP_LOGE(TAG, "IMU Encoder I2C bus is not initialized.");
        }
        return imu_encoder_i2c_bus_;
    } else if (num == ENCODER_I2C_BUS) {
        if (encoder_i2c_bus_ == nullptr) {
            ESP_LOGE(TAG, "Encoder I2C bus is not initialized.");
        }
        return encoder_i2c_bus_;
    } else {
        ESP_LOGE(TAG, "Unknown I2C bus number.");
        return nullptr;
    }
}

void HAL::Init()
{
    init_i2c();
    ESP_LOGI(TAG, "init mpu...");
    imu_init();
    ESP_LOGI(TAG, "init motor...");
    motor_init();
}
