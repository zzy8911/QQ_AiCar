/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mt6701.h"
#include "math.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "MT6701";
static constexpr float M_2PI = 6.28318530717958647692f;
static constexpr float RAW_TO_RAD = M_2PI / 16384.0f;
#define INIT_SPI_BUS_INTERNAL   1

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
    _spi_host = spi_host;
    _sclk_io = sclk_io;
    _miso_io = miso_io;
    _mosi_io = mosi_io;
    _cs_io = cs_io;
    _is_installed = false;
}

MT6701::MT6701(i2c_port_t i2c_port, gpio_num_t sclk_io, gpio_num_t miso_io)
{
    _i2c_port = i2c_port;
    _sclk_io = sclk_io;
    _miso_io = miso_io;
    _is_installed = false;
}

MT6701::~MT6701()
{
    if (_is_installed) {
        deinit();
    }
}

void MT6701::init()
{
    esp_err_t ret;

    if (_spi_host != SPI_HOST_MAX) {
#if INIT_SPI_BUS_INTERNAL
        // Configuration for the spi bus
        spi_bus_config_t buscfg = {
            .mosi_io_num = _mosi_io,
            .miso_io_num = _miso_io,
            .sclk_io_num = _sclk_io,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 16,
        };

        ret = spi_bus_initialize(_spi_host, &buscfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK) {
            ESP_LOGI(TAG, "SPI bus init failed");
        }
#endif
        spi_device_interface_config_t dev_cfg = {
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 3,
            .clock_speed_hz = 4000000,
            .spics_io_num = _cs_io,
            .flags = 0,
            .queue_size = 7,
            .pre_cb = NULL,
            .post_cb = NULL,
        };
        ret = spi_bus_add_device(_spi_host, &dev_cfg, &_spi_device);
        ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "SPI bus add device fail");

        memset(&_trans, 0, sizeof(_trans));
        _trans.flags = SPI_TRANS_USE_RXDATA; // 标记只接收数据
        _trans.length = 24;                  // 总线传输长度（16 bits）
        _trans.rxlength = 24;                // 接收长度
        // .tx_buffer 和 .rx_buffer 保持 NULL，使用 inline 接收

        _is_reading_started = false;
        _last_angle = 0.0f;

        _is_installed = true;
    } else if (_i2c_port != I2C_NUM_MAX) {
        // Configuration for the i2c bus
        i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = _miso_io,
            .scl_io_num = _sclk_io,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
        };
        i2c_config.master.clk_speed = 400 * 1000;

        _i2c_bus = i2c_bus_create(_i2c_port, &i2c_config);
        ESP_RETURN_ON_FALSE(_i2c_bus != NULL,, TAG, "I2C bus create fail");
        _i2c_device = i2c_bus_device_create(_i2c_bus, 0x06, 0);
        ESP_RETURN_ON_FALSE(_i2c_device != NULL,, TAG, "MT6701 device create fail");

        _is_installed = true;
    } else {
        _is_installed = false;
    }
}

void MT6701::deinit()
{
    esp_err_t ret;

    if (_spi_host != SPI_HOST_MAX) {
        ret = spi_bus_remove_device(_spi_device);
        ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "SPI remove device fail");
#if INIT_SPI_BUS_INTERNAL
        ret = spi_bus_free(_spi_host);
        ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "SPI free fail");
#endif
        _is_installed = false;
    } else if (_i2c_port != I2C_NUM_MAX) {
        i2c_bus_device_delete(&_i2c_device);
        ESP_RETURN_ON_FALSE(_i2c_device == NULL,, TAG, "MT6701 device delete fail");
        ret = i2c_bus_delete(&_i2c_bus);
        ESP_RETURN_ON_FALSE(ret == ESP_OK,, TAG, "I2C bus delete fail");
        _is_installed = false;
    }
}

float MT6701::getSensorAngle()
{
    esp_err_t ret;

    if (_spi_host != SPI_HOST_MAX) {
        // 1. 如果不是第一次读取，尝试获取上次 DMA 传输的结果 (非阻塞)
        if (_is_reading_started) {
            spi_transaction_t *r_trans  = nullptr;

            // 尝试获取结果，等待时间设置为 0ms，实现非阻塞
            ret = spi_device_get_trans_result(_spi_device, &r_trans, 0);
            if (ret == ESP_OK) {
                // 成功：获取到数据，更新 _last_angle
                uint32_t spi_data = ((uint32_t)r_trans->rx_data[0] << 16) | ((uint32_t)r_trans->rx_data[1]<<8) | ((uint32_t)r_trans->rx_data[2]);
                uint32_t angle_spi = spi_data >> 10;

                _last_angle = (float)angle_spi * RAW_TO_RAD;
            } else if (ret != ESP_ERR_TIMEOUT) {
                // 发生错误 (非超时，超时是正常的挂起状态)
                // ESP_LOGE(TAG, "SPI get result failed: %s", esp_err_to_name(ret));
            }
        } else {
            // 第一次调用：使用阻塞模式进行初始化读取，并填充 _last_angle
            ret = spi_device_polling_transmit(_spi_device, &_trans);
            if (ret == ESP_OK) {
                uint32_t spi_data = ((uint32_t)_trans.rx_data[0] << 16) | ((uint32_t)_trans.rx_data[1]<<8) | ((uint32_t)_trans.rx_data[2]);
                uint32_t angle_spi = spi_data >> 10;

                _last_angle = (float)angle_spi * RAW_TO_RAD;
                _is_reading_started = true;
            } else {
                ESP_LOGE(TAG, "Initial SPI read failed: %s", esp_err_to_name(ret));
                return -1; // 初始化失败
            }
        }

        // 2. 排队启动下一次 DMA 传输
        // 使用 queue_trans 而非 polling_transmit，实现非阻塞
        ret = spi_device_queue_trans(_spi_device, &_trans, 0);
        if (ret != ESP_OK) {
            // ESP_LOGE(TAG, "SPI queue failed: %s", esp_err_to_name(ret));
        }

        return _last_angle; // 返回本次循环中获取或继承的最新角度
    } else if (_i2c_port != I2C_NUM_MAX) {
        uint8_t raw_angle_buffer[2] = {0};
        if (i2c_bus_read_bytes(_i2c_device, 0x03, 2, raw_angle_buffer) != ESP_OK) {
            return -1;
        }
        _last_angle = ((int)((raw_angle_buffer[0] << 6) | (raw_angle_buffer[1] >> 2))) * 0.00038349519f; /*!< Converts raw data into angle information in radians. */
        return _last_angle;
    }

    return -1;
}