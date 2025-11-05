#include "BMI270.h"
#include <cstring>
#include <cmath>
#include <cstdio>

#define GRAVITY_EARTH (9.80665f)

#define TAG "BMI270"

BMI270::BMI270(i2c_master_bus_handle_t i2c_handle, SemaphoreHandle_t i2c_semaphore, uint8_t address)
    : i2c_handle_(i2c_handle), i2c_semaphore_(i2c_semaphore), address_(address) {
    memset(&dev_, 0, sizeof(dev_));
    memset(&sensor_data_, 0, sizeof(sensor_data_));
}

int BMI270::init() {
    int8_t rslt;

    // 初始化 I2C 接口
    bmi2_set_i2c_configuration(i2c_handle_, address_, i2c_semaphore_);

    rslt = bmi2_interface_init(&dev_, BMI2_I2C_INTF);
    if (rslt != BMI2_OK) return rslt;

    rslt = bmi270_init(&dev_);
    if (rslt != BMI2_OK) return rslt;

    rslt = setGyroConfig(BMI2_GYR_ODR_400HZ, BMI2_GYR_RANGE_500);
    if (rslt != BMI2_OK) return rslt;

    rslt = setAccelConfig(BMI2_ACC_ODR_400HZ, BMI2_ACC_RANGE_2G);
    if (rslt != BMI2_OK) return rslt;

    uint8_t sensor_list[] = {BMI2_GYRO, BMI2_ACCEL};
    rslt = bmi2_sensor_enable(sensor_list, sizeof(sensor_list), &dev_);
    if (rslt != BMI2_OK) return rslt;

    ESP_LOGI(TAG, "BMI270 init success.");
    return rslt;
}

int8_t BMI270::setGyroConfig(uint8_t odr, uint8_t range) {
    int8_t                  rslt;
    struct bmi2_sens_config config;
    config.type = BMI2_GYRO;
    rslt        = bmi2_get_sensor_config(&config, 1, &dev_);
    bmi2_error_codes_print_result(rslt);
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, &dev_);
    bmi2_error_codes_print_result(rslt);
    if (rslt == BMI2_OK) {
        config.cfg.gyr.odr         = odr;
        config.cfg.gyr.range       = range;
        config.cfg.gyr.bwp         = BMI2_GYR_NORMAL_MODE;
        config.cfg.gyr.noise_perf  = BMI2_POWER_OPT_MODE;
        config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
        rslt                       = bmi2_set_sensor_config(&config, 1, &dev_);
    }
    return rslt;
}

int8_t BMI270::setAccelConfig(uint8_t odr, uint8_t range) {
    int8_t rslt;

    struct bmi2_sens_config config;
    config.type = BMI2_ACCEL;

    rslt = bmi2_get_sensor_config(&config, 1, &dev_);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        config.cfg.acc.odr         = odr;
        config.cfg.acc.range       = range;
        config.cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4;
        config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        rslt = bmi2_set_sensor_config(&config, 1, &dev_);
        bmi2_error_codes_print_result(rslt);

        rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &dev_);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

float BMI270::lsbToDps(int16_t val, float dps, uint8_t bit_width) {
    float half_scale = powf(2, bit_width) / 2.0f;
    return (dps / half_scale) * val;
}

float BMI270::lsbToMps2(int16_t val, float g_range, uint8_t bit_width) {
    float half_scale = powf(2, bit_width) / 2.0f;
    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

int BMI270::update() {
    int8_t rslt = bmi2_get_sensor_data(&sensor_data_, &dev_);
    if (rslt != BMI2_OK) return -1;

    acc_x_ = lsbToMps2(sensor_data_.acc.x, 2.0f, dev_.resolution);
    acc_y_ = lsbToMps2(sensor_data_.acc.y, 2.0f, dev_.resolution);
    acc_z_ = lsbToMps2(sensor_data_.acc.z, 2.0f, dev_.resolution);

    gyr_x_ = lsbToDps(sensor_data_.gyr.x, 2000.0f, dev_.resolution);
    gyr_y_ = lsbToDps(sensor_data_.gyr.y, 2000.0f, dev_.resolution);
    gyr_z_ = lsbToDps(sensor_data_.gyr.z, 2000.0f, dev_.resolution);

    return ESP_OK;
}
