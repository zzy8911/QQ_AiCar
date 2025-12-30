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

int BMI270::init(i2c_master_bus_handle_t i2c_handle, SemaphoreHandle_t i2c_semaphore) {
    int8_t rslt;

    if (i2c_handle)
        i2c_handle_ = i2c_handle;
    if (i2c_semaphore)
        i2c_semaphore_ = i2c_semaphore;

    if (i2c_handle_ == nullptr) {
        ESP_LOGE(TAG, "I2C handle is null.");
        return -1;
    }

    // 初始化 I2C 接口
    bmi2_set_i2c_configuration(i2c_handle_, address_, i2c_semaphore_);

    rslt = bmi2_interface_init(&dev_, BMI2_I2C_INTF);
    ESP_LOGI(TAG, "BMI270 interface init result: %d", rslt);
    if (rslt != BMI2_OK) return rslt;

    rslt = bmi270_init(&dev_);
    ESP_LOGI(TAG, "BMI270 device init result: %d", rslt);
    if (rslt != BMI2_OK) return rslt;

    rslt = setGyroConfig(BMI2_GYR_ODR_800HZ, BMI2_GYR_RANGE_500);
    ESP_LOGI(TAG, "BMI270 gyro config result: %d", rslt);
    if (rslt != BMI2_OK) return rslt;

    rslt = setAccelConfig(BMI2_ACC_ODR_800HZ, BMI2_ACC_RANGE_2G);
    ESP_LOGI(TAG, "BMI270 accel config result: %d", rslt);
    if (rslt != BMI2_OK) return rslt;

    initScales(500, 2.0); // 500 dps, 2g

    uint8_t sensor_list[] = {BMI2_GYRO, BMI2_ACCEL};
    rslt = bmi2_sensor_enable(sensor_list, sizeof(sensor_list), &dev_);
    ESP_LOGI(TAG, "BMI270 sensor enable result: %d", rslt);
    if (rslt != BMI2_OK) return rslt;

    // Disable advanced power save mode
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &dev_);
    ESP_LOGI(TAG, "BMI270 APS disable result: %d", rslt);
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

void BMI270::initScales(float gyro_range, float acc_range) {
    // 2^resolution = 65536, half-range = 32768
    float half_scale = (1 << dev_.resolution) * 0.5f;

    acc_scale_ = (GRAVITY_EARTH * acc_range) / half_scale;  // m/s^2 per LSB
    gyr_scale_ = gyro_range / half_scale;                   // dps per LSB
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

    acc_x_ = sensor_data_.acc.x * acc_scale_;
    acc_y_ = sensor_data_.acc.y * acc_scale_;
    acc_z_ = sensor_data_.acc.z * acc_scale_;

    gyr_x_ = sensor_data_.gyr.x * gyr_scale_;
    gyr_y_ = sensor_data_.gyr.y * gyr_scale_;
    gyr_z_ = sensor_data_.gyr.z * gyr_scale_;

    // ESP_LOGI(TAG, "acc: %.2f, %.2f, %.2f m/s²; gyr: %.2f, %.2f, %.2f dps",
    //          acc_x_, acc_y_, acc_z_, gyr_x_, gyr_y_, gyr_z_);

    return ESP_OK;
}
