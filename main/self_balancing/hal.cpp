
#include "hal.h"
#include "boards/self-balancing-robot/config.h"
#include "motor.h"
#include "controller.h"
#include "board.h"
#include "port/esp32_adc_driver.h"

#define TAG "HAL"

// This function uses BLE, which can interfere with SoftAP.
void HAL::InitController()
{
    static bool initialized = false;
    if (initialized) {
        ESP_LOGW(TAG, "HAL controller already initialized, skipping.");
        return;
    }

    // get ble addr from nvs
    std::string qqcar_ble_addr;
    nvs_handle_t nvs;
    if (nvs_open("qqcar", NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = 0;
        if (nvs_get_str(nvs, "ble_addr", nullptr, &len) == ESP_OK && len > 1) {
            qqcar_ble_addr.resize(len - 1);
            nvs_get_str(nvs, "ble_addr", qqcar_ble_addr.data(), &len);
            ESP_LOGI(TAG, "QQCar BLE addr: %s", qqcar_ble_addr.c_str());
        }
        nvs_close(nvs);
    }
    controller_init(qqcar_ble_addr.empty() ? DEFAULTU_BLE_ADDR : qqcar_ble_addr.c_str());

    initialized = true;
}

void HAL::InitMotionSystem(i2c_master_bus_handle_t i2c_bus)
{
    static bool initialized = false;
    if (initialized) {
        ESP_LOGW(TAG, "HAL already initialized, skipping.");
        return;
    }

    // imu init
    auto imu = std::make_shared<Imu>(
        i2c_bus,
        nullptr,
        FilterType::MAHONY,
        CoordinateSystem::X_FORWARD
    );
    imu->init();

    // battery estimation adc init
    adcInit(BATTERY_ADC_GPIO);

    // motor init
    ESP_LOGI(TAG, "init motor...");
    auto& motor = Motor::getInstance();
    motor.attachImu(imu);
    motor.init();

    initialized = true;
}
