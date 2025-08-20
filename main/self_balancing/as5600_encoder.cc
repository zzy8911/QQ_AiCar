#include "as5600_encoder.h"

#define TAG     "AS5600Encoder"

AS5600Encoder::AS5600Encoder(i2c_master_bus_handle_t i2c_bus)
    : i2c_bus_(i2c_bus) {}

AS5600Encoder::~AS5600Encoder() {
    deinit();
}

void AS5600Encoder::init() {
    if (device_ == nullptr) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = AS5600_I2C_ADDR,
            .scl_speed_hz = 400000,
        };

        esp_err_t ret = i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &device_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "AS5600 I2C initialized successfully");
        }
    }
}

void AS5600Encoder::deinit() {
    if (device_ != nullptr) {
        esp_err_t ret = i2c_master_bus_rm_device(device_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "I2C device removed successfully");
        }
        device_ = nullptr;
    }
}

float AS5600Encoder::getSensorAngle() {
    uint8_t raw_angle_buf[2] = {0};
    uint8_t reg = ANGLE_REG;

    esp_err_t ret = i2c_master_transmit_receive(device_, &reg, 1, raw_angle_buf, 2, 100);
    if (ESP_OK == ret) {
        uint16_t raw_angle = (raw_angle_buf[0] << 8) | raw_angle_buf[1];
        float angle = (raw_angle & 0x0FFF) / 4096.0f * (2 * PI);
        return angle;
    } else {
        ESP_LOGE(TAG, "Failed to read angle data");
    }

    return 0.0f;
}