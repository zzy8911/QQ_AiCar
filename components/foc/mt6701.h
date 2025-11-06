/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "i2c_bus.h"

class MT6701 {
public:
    /**
     * @brief Construct a new mt6701 object
     *
     * @param spi_host
     * @param sclk_io
     * @param miso_io
     * @param mosi_io
     * @param cs_io
     */
    MT6701(spi_host_device_t spi_host, gpio_num_t sclk_io, gpio_num_t miso_io, gpio_num_t mosi_io, gpio_num_t cs_io);

    /**
     * @brief Construct a new MT6701 object
     *
     * @param i2c_port
     * @param sclk_io
     * @param miso_io
     */
    MT6701(i2c_port_t i2c_port, gpio_num_t sclk_io, gpio_num_t miso_io);

    /**
     * @brief Destroy the mt6701 object
     *
     */
    ~MT6701();

    /**
     * @brief Init spi for mt6701
     *
     */
    void init();

    /**
     * @brief Deinit spi for mt6701
     *
     */
    void deinit();

    /**
     * @brief Get the output of mt6701
     *
     * @return float
     */
    float getSensorAngle();

    void update();
    float getAngle();
    float getMechanicalAngle();
    float getVelocity();

private:
    i2c_bus_handle_t i2c_bus_;
    i2c_bus_device_handle_t i2c_device_;
    i2c_port_t i2c_port_ = I2C_NUM_MAX;

    spi_host_device_t spi_host_ = SPI_HOST_MAX;
    spi_device_handle_t spi_device_;
    gpio_num_t sclk_io_;
    gpio_num_t miso_io_;
    gpio_num_t mosi_io_;
    gpio_num_t cs_io_;

    bool is_installed_;
    int full_rotations_ = 0;
    float angle_prev_ = 0;
    int vel_full_rotations_ = 0;
    float vel_angle_prev_ = 0;
    int64_t angle_prev_ts_ = 0;
    int64_t vel_angle_prev_ts_ = 0;
};
