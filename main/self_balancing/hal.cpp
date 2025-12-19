
#include "hal.h"
#include "boards/self-balancing-robot/config.h"
#include "motor.h"
#include "controller.h"
#include "board.h"
#include "port/esp32_adc_driver.h"

#define TAG "HAL"

void HAL::Init(i2c_master_bus_handle_t i2c_bus)
{
    controller_init(DEFAULTU_BLE_ADDR);

    auto imu = std::make_shared<Imu>(
        i2c_bus,
        nullptr,
        FilterType::MAHONY,
        CoordinateSystem::X_FORWARD
    );
    imu->init();

    // battery
    adcInit(BATTERY_ADC_GPIO);

    ESP_LOGI(TAG, "init motor...");
    auto& motor = Motor::getInstance();
    motor.attachImu(imu);
    motor.init();
}
