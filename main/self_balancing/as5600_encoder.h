#pragma once

#include "hal.h"
#include "common/base_classes/Sensor.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

class AS5600Encoder : public Sensor {
public:
    AS5600Encoder(I2C_BUS i2c_bus_id);
    ~AS5600Encoder();
    void init() override;
    void deinit();
    float getSensorAngle() override;

private:
    i2c_master_dev_handle_t device_ = nullptr;
    I2C_BUS i2c_bus_id_;
    static constexpr uint8_t AS5600_I2C_ADDR = 0x36;
    static constexpr uint8_t ANGLE_REG = 0x0C;
};