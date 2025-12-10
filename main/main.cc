#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_event.h>

#include "application.h"
#include "system_info.h"

#define TAG "main"

#ifdef CONFIG_ENABLE_CONSOLE
#include "esp_console.h"
esp_err_t console_init(esp_console_repl_t **ret_repl, const char* prompt)
{
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = prompt;
    repl_config.max_cmdline_length = 128;

    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    return esp_console_new_repl_uart(&hw_config, &repl_config, ret_repl);
}
#endif

extern "C" void app_main(void)
{
    // Initialize the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS flash for WiFi configuration
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash to fix corruption");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#ifdef CONFIG_ENABLE_CONSOLE
    esp_console_repl_t *repl = NULL;
    console_init(&repl, "QQBot>");
    esp_console_register_help_command();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
#endif

    // Launch the application
    Application::GetInstance().Start();
}
