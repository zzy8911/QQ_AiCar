#ifndef __HAL_H__
#define __HAL_H__
// #include "button_event.h"
// #include "CommonMacro.h"
#include <esp_log.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_platform.h"

namespace HAL
{
    void Init(i2c_master_bus_handle_t i2c_bus);
}
#endif