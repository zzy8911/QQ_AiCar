
#include "hal.h"
#include "boards/self-balancing-robot/config.h"
#include "motor.h"
#include "controller.h"
#include "board.h"

#define TAG "HAL"

i2c_master_bus_handle_t HAL::get_i2c_bus()
{
    return Board::GetInstance().GetI2cBus();
}

void HAL::Init()
{
    controller_init(DEFAULTU_BLE_ADDR);

    ESP_LOGI(TAG, "init motor...");
    Motor::getInstance().init();
}
