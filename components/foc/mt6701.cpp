/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mt6701.h"
#include "math.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "MT6701";
#define _2PI 6.28318530718f

/* Constants for CRC calculation */
static uint8_t tableCRC6[64] = {
    0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
    0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
    0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
    0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
    0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
    0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
    0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
    0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02
};

/* 32-bit input data, right alignment, Calculation over 18 bits (mult. of 6) */
static uint8_t CRC6_43_18bit(uint32_t w_InputData)
{
    uint8_t b_Index = 0;
    uint8_t b_CRC = 0;

    b_Index = (uint8_t)(((uint32_t)w_InputData >> 12u) & 0x0000003Fu);

    b_CRC = (uint8_t)(((uint32_t)w_InputData >> 6u) & 0x0000003Fu);
    b_Index = b_CRC ^ tableCRC6[b_Index];

    b_CRC = (uint8_t)((uint32_t)w_InputData & 0x0000003Fu);
    b_Index = b_CRC ^ tableCRC6[b_Index];

    b_CRC = tableCRC6[b_Index];

    return b_CRC;
}

MT6701::MT6701(spi_host_device_t spi_host, gpio_num_t sclk_io, gpio_num_t miso_io, gpio_num_t mosi_io, gpio_num_t cs_io)
{
    spi_host_ = spi_host;
    sclk_io_ = sclk_io;
    miso_io_ = miso_io;
    mosi_io_ = mosi_io;
    cs_io_ = cs_io;

    is_installed_ = false;
}

MT6701::MT6701(i2c_port_t i2c_port, gpio_num_t sclk_io, gpio_num_t miso_io)
{
    i2c_port_ = i2c_port;
    sclk_io_ = sclk_io;
    miso_io_ = miso_io;
    is_installed_ = false;
}

MT6701::~MT6701()
{
    if (is_installed_) {
        deinit();
    }
}

void MT6701::init()
{
    esp_err_t ret;

    if (spi_host_ != SPI_HOST_MAX) {
        spi_device_interface_config_t dev_cfg = {
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 0,
            .clock_speed_hz = 1000000,
            .input_delay_ns = 70,
            .spics_io_num = cs_io_,
            .flags = 0,
            .queue_size = 1,
            .pre_cb = NULL,
            .post_cb = NULL,
        };

        ret = spi_bus_add_device(spi_host_, &dev_cfg, &spi_device_);
        ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "SPI bus add device fail");

        is_installed_ = true;
    } else if (i2c_port_ != I2C_NUM_MAX) {
        // Configuration for the i2c bus
        i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = miso_io_,
            .scl_io_num = sclk_io_,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
        };
        i2c_config.master.clk_speed = 400 * 1000;

        i2c_bus_ = i2c_bus_create(i2c_port_, &i2c_config);
        ESP_RETURN_ON_FALSE(i2c_bus_ != NULL,, TAG, "I2C bus create fail");
        i2c_device_ = i2c_bus_device_create(i2c_bus_, 0x06, 0);
        ESP_RETURN_ON_FALSE(i2c_device_ != NULL,, TAG, "MT6701 device create fail");

        is_installed_ = true;
    } else {
        is_installed_ = false;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    getSensorAngle();
    esp_rom_delay_us(1);
    vel_angle_prev_ = getSensorAngle();
    vel_angle_prev_ts_ = esp_timer_get_time();

    vTaskDelay(pdMS_TO_TICKS(1));
    getSensorAngle();
    esp_rom_delay_us(1);
    angle_prev_ = getSensorAngle();
    angle_prev_ts_ = esp_timer_get_time();
}

void MT6701::deinit()
{
    esp_err_t ret;

    if (spi_host_ != SPI_HOST_MAX) {
        ret = spi_bus_remove_device(spi_device_);
        ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "SPI remove device fail");
        // ret = spi_bus_free(spi_host_);
        // ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "SPI free fail");
        is_installed_ = false;
    } else if (i2c_port_ != I2C_NUM_MAX) {
        i2c_bus_device_delete(&i2c_device_);
        ESP_RETURN_ON_FALSE(i2c_device_ == NULL,, TAG, "MT6701 device delete fail");
        ret = i2c_bus_delete(&i2c_bus_);
        ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "I2C bus delete fail");
        is_installed_ = false;
    }
}

float MT6701::getSensorAngle()
{
    esp_err_t ret;
    static float angle = 0.0;
    static float previous_angle = 0.0;

    if (spi_host_ != SPI_HOST_MAX) {
        spi_transaction_t spi_transaction = {
            .flags = SPI_TRANS_USE_RXDATA,
            .length = 24,
            .rxlength = 24,
            .tx_buffer = NULL,
            .rx_buffer = NULL,
        };
        ret = spi_device_polling_transmit(spi_device_, &spi_transaction);
        ESP_RETURN_ON_FALSE(ret == ESP_OK, 0.0, TAG, "SPI transaction failed: %s", esp_err_to_name(ret));

        uint32_t spi_32 = ((int32_t)spi_transaction.rx_data[0] << 16) | ((int32_t)spi_transaction.rx_data[1] << 8) | spi_transaction.rx_data[2];
        uint32_t angle_spi = spi_32 >> 10;

        uint8_t received_crc = spi_32 & 0x3F;
        uint8_t calculated_crc = CRC6_43_18bit(spi_32 >> 6);

        if (received_crc == calculated_crc) {
            angle = (float)angle_spi * 2 * M_PI / 16384;
            previous_angle = angle;
            // ESP_LOGI(TAG, "Angle:%f", angle);
        } else {
            return previous_angle;
        }
        return angle;
    } else if (i2c_port_ != I2C_NUM_MAX) {
        uint8_t raw_angle_buffer[2] = {0};
        if (i2c_bus_read_bytes(i2c_device_, 0x03, 2, raw_angle_buffer) != ESP_OK) {
            return -1;
        }
        angle = ((int)((raw_angle_buffer[0] << 6) | (raw_angle_buffer[1] >> 2))) * 0.00038349519f; /*!< Converts raw data into angle information in radians. */
        return angle;
    }

    return -1;
}

void MT6701::update() {
    float val = getSensorAngle();
    angle_prev_ts_ = esp_timer_get_time();
    float d_angle = val - angle_prev_;
    if (abs(d_angle) > (0.8f * _2PI)) full_rotations_ += (d_angle > 0) ? -1 : 1;
    angle_prev_ = val;
}

float MT6701::getMechanicalAngle() {
    // ESP_LOGI(TAG, "Mech Angle: %f", angle_prev_);
    return angle_prev_;
}

float MT6701::getAngle() {
    return (float)full_rotations_ * _2PI + angle_prev_;
}

float MT6701::getVelocity() {
    float Ts = (angle_prev_ts_ - vel_angle_prev_ts_) * 1e-6;
    if (Ts <= 0) Ts = 1e-3f;
    float vel = ((float)(full_rotations_ - vel_full_rotations_) * _2PI + (angle_prev_ - vel_angle_prev_)) / Ts;
    vel_angle_prev_ = angle_prev_;
    vel_full_rotations_ = full_rotations_;
    vel_angle_prev_ts_ = angle_prev_ts_;
    return vel;
}
