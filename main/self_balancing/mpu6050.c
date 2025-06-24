/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "mpu6050.h"

#define TAG "MPU6050"

const uint8_t MPU6050_DATA_RDY_INT_BIT =      (uint8_t) BIT0;
const uint8_t MPU6050_I2C_MASTER_INT_BIT =    (uint8_t) BIT3;
const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT = (uint8_t) BIT4;
const uint8_t MPU6050_MOT_DETECT_INT_BIT =    (uint8_t) BIT6;
const uint8_t MPU6050_ALL_INTERRUPTS = (MPU6050_DATA_RDY_INT_BIT | MPU6050_I2C_MASTER_INT_BIT | MPU6050_FIFO_OVERFLOW_INT_BIT | MPU6050_MOT_DETECT_INT_BIT);

static esp_err_t mpu6050_write(mpu6050_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    esp_err_t ret;

    uint8_t *write_buf = malloc(data_len + 1);
    if (!write_buf) {
        ESP_LOGE(TAG, "Malloc write buffer failed");
        return ESP_ERR_NO_MEM;
    }

    write_buf[0] = reg_start_addr;
    memcpy(write_buf + 1, data_buf, data_len);

    ret = i2c_master_transmit(sens->dev, write_buf, data_len + 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }

    free(write_buf);

    return ret;
}

static esp_err_t mpu6050_read(mpu6050_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    esp_err_t ret;

    // Write register address
    ret = i2c_master_transmit(sens->dev, &reg_start_addr, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write register address failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read data
    ret = i2c_master_receive(sens->dev, data_buf, data_len, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

mpu6050_handle_t mpu6050_create(i2c_master_bus_handle_t i2c_bus, const uint16_t dev_addr)
{
    mpu6050_dev_t *sensor = (mpu6050_dev_t *) calloc(1, sizeof(mpu6050_dev_t));

    // 配置设备
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = 400000,
    };
    // 创建设备句柄
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &sensor->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        // i2c_del_master_bus(sensor->bus);
        free(sensor);
        return NULL;
    }
    
    sensor->dev_addr = dev_addr;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (mpu6050_handle_t) sensor;
}

void mpu6050_delete(mpu6050_handle_t sensor)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    if (sens->timer) {
        free(sens->timer);
    }
    if (sens->dev) {
        i2c_master_bus_rm_device(sens->dev);
    }
    // if (sens->bus) {
    //     i2c_del_master_bus(sens->bus);
    // }
    free(sens);
}

void mpu6050_calc_gyro_offsets(mpu6050_handle_t sensor, bool console, uint16_t delayBefore, uint16_t delayAfter)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    float x = 0, y = 0, z = 0;
    int16_t rx, ry, rz;
    uint8_t data[6];
    const int samples = 3000;
    esp_err_t ret;

    vTaskDelay(pdMS_TO_TICKS(delayBefore));
    if (console) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Calculating gyro offsets");
        ESP_LOGI(TAG, "DO NOT MOVE MPU6050");
    }

    for (int i = 0; i < samples; i++) {
        ret = mpu6050_read(sensor, MPU6050_GYRO_XOUT_H, data, 6);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyro data: %s", esp_err_to_name(ret));
            continue;
        }

        rx = (int16_t)((data[0] << 8) | data[1]);
        ry = (int16_t)((data[2] << 8) | data[3]);
        rz = (int16_t)((data[4] << 8) | data[5]);

        x += ((float)rx) / 65.5f;
        y += ((float)ry) / 65.5f;
        z += ((float)rz) / 65.5f;

        if (console && i % 1000 == 0) {
            ESP_LOGI(TAG, ".");
        }
    }

    sens->gyroXoffset = x / samples;
    sens->gyroYoffset = y / samples;
    sens->gyroZoffset = z / samples;

    if (console) {
        ESP_LOGI(TAG, "Done!");
        ESP_LOGI(TAG, "X offset : %.2f", sens->gyroXoffset);
        ESP_LOGI(TAG, "Y offset : %.2f", sens->gyroYoffset);
        ESP_LOGI(TAG, "Z offset : %.2f", sens->gyroZoffset);
        ESP_LOGI(TAG, "Program will start after %d ms", delayAfter);
        ESP_LOGI(TAG, "========================================");
    }
    vTaskDelay(pdMS_TO_TICKS(delayAfter));
}

esp_err_t mpu6050_get_deviceid(mpu6050_handle_t sensor, uint8_t *const deviceid)
{
    return mpu6050_read(sensor, MPU6050_WHO_AM_I, deviceid, 1);
}

esp_err_t mpu6050_wake_up(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp &= (~BIT6);
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_sleep(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT6;
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_config(mpu6050_handle_t sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t config_regs[2] = {gyro_fs << 3,  acce_fs << 3};
    return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs;
    ret = mpu6050_read(sensor, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
    case ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity)
{
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = mpu6050_read(sensor, MPU6050_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
    case GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;

    case GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;

    case GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;

    case GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_config_interrupts(mpu6050_handle_t sensor, const mpu6050_int_config_t *const interrupt_configuration)
{
    esp_err_t ret = ESP_OK;

    if (NULL == interrupt_configuration) {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    if (GPIO_IS_VALID_GPIO(interrupt_configuration->interrupt_pin)) {
        // Set GPIO connected to MPU6050 INT pin only when user configures interrupts.
        mpu6050_dev_t *sensor_device = (mpu6050_dev_t *) sensor;
        sensor_device->int_pin = interrupt_configuration->interrupt_pin;
    } else {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    uint8_t int_pin_cfg = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
        int_pin_cfg |= BIT7;
    }

    if (INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode) {
        int_pin_cfg |= BIT6;
    }

    if (INTERRUPT_LATCH_UNTIL_CLEARED == interrupt_configuration->interrupt_latch) {
        int_pin_cfg |= BIT5;
    }

    if (INTERRUPT_CLEAR_ON_ANY_READ == interrupt_configuration->interrupt_clear_behavior) {
        int_pin_cfg |= BIT4;
    }

    ret = mpu6050_write(sensor, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    gpio_int_type_t gpio_intr_type;

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
        gpio_intr_type = GPIO_INTR_NEGEDGE;
    } else {
        gpio_intr_type = GPIO_INTR_POSEDGE;
    }

    gpio_config_t int_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = gpio_intr_type,
        .pin_bit_mask = (BIT0 << interrupt_configuration->interrupt_pin)
    };

    ret = gpio_config(&int_gpio_config);

    return ret;
}

esp_err_t mpu6050_register_isr(mpu6050_handle_t sensor, const mpu6050_isr_t isr)
{
    esp_err_t ret;
    mpu6050_dev_t *sensor_device = (mpu6050_dev_t *) sensor;

    if (NULL == sensor_device) {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    ret = gpio_isr_handler_add(
              sensor_device->int_pin,
              ((gpio_isr_t) * (isr)),
              ((void *) sensor)
          );

    if (ESP_OK != ret) {
        return ret;
    }

    ret = gpio_intr_enable(sensor_device->int_pin);

    return ret;
}

esp_err_t mpu6050_enable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    if (enabled_interrupts != interrupt_sources) {

        enabled_interrupts |= interrupt_sources;

        ret = mpu6050_write(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6050_disable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources)
{
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    if (0 != (enabled_interrupts & interrupt_sources)) {
        enabled_interrupts &= (~interrupt_sources);

        ret = mpu6050_write(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6050_get_interrupt_status(mpu6050_handle_t sensor, uint8_t *const out_intr_status)
{
    esp_err_t ret;

    if (NULL == out_intr_status) {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    ret = mpu6050_read(sensor, MPU6050_INTR_STATUS, out_intr_status, 1);

    return ret;
}

inline uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status)
{
    return (MPU6050_DATA_RDY_INT_BIT == (MPU6050_DATA_RDY_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status)
{
    return (uint8_t) (MPU6050_I2C_MASTER_INT_BIT == (MPU6050_I2C_MASTER_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status)
{
    return (uint8_t) (MPU6050_FIFO_OVERFLOW_INT_BIT == (MPU6050_FIFO_OVERFLOW_INT_BIT & interrupt_status));
}

esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t sensor, mpu6050_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

    return ret;
}

esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acce_value_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce;

    ret = mpu6050_get_acce_sensitivity(sensor, &acce_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6050_get_raw_acce(sensor, &raw_acce);
    if (ret != ESP_OK) {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro;

    ret = mpu6050_get_gyro_sensitivity(sensor, &gyro_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6050_get_raw_gyro(sensor, &raw_gyro);
    if (ret != ESP_OK) {
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_temp(mpu6050_handle_t sensor, mpu6050_temp_value_t *const temp_value)
{
    uint8_t data_rd[2];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_TEMP_XOUT_H, data_rd, sizeof(data_rd));
    temp_value->temp = (int16_t)((data_rd[0] << 8) | (data_rd[1])) / 340.00 + 36.53;
    return ret;
}

esp_err_t mpu6050_complimentory_filter(mpu6050_handle_t sensor, const mpu6050_acce_value_t *const acce_value,
                                       const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;

    sens->counter++;
    if (sens->counter == 1) {
        acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
        complimentary_angle->roll = acce_angle[0];
        complimentary_angle->pitch = acce_angle[1];
        gettimeofday(sens->timer, NULL);
        return ESP_OK;
    }

    struct timeval now, dt_t;
    gettimeofday(&now, NULL);
    timersub(&now, sens->timer, &dt_t);
    sens->dt = (float) (dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
    gettimeofday(sens->timer, NULL);

    acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);

    gyro_rate[0] = gyro_value->gyro_x;
    gyro_rate[1] = gyro_value->gyro_y;
    gyro_angle[0] = gyro_rate[0] * sens->dt;
    gyro_angle[1] = gyro_rate[1] * sens->dt;

    complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

    return ESP_OK;
}