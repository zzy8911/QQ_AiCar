
#include "hal.h"
#include "boards/self-balancing-robot/config.h"

#define TAG "HAL"

static i2c_master_bus_handle_t i2c0_bus_;  // icm42688, as5600
static i2c_master_bus_handle_t i2c1_bus_;  // as5600

static void init_i2c() {
    // Initialize I2C peripheral
    i2c_master_bus_config_t i2c0_bus_cfg = {
        .i2c_port = (i2c_port_t)I2C_NUM_0,
        .sda_io_num = IMU_SDA,
        .scl_io_num = IMU_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c0_bus_cfg, &i2c0_bus_));

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
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c1_bus_cfg, &i2c1_bus_));
}

i2c_master_bus_handle_t HAL::get_i2c_bus(I2C_BUS num)
{
    if (num == IMU_BUS) {
        return i2c0_bus_;
    } else if (num == ENCODER_BUS) {
        return i2c1_bus_;
    } else {
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
