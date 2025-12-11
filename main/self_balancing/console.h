#pragma once

#include "esp_err.h"
#include "esp_console.h"
#include "esp_log.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化并启动 Console REPL
 *
 * @param prompt 控制台提示符，如 "QQBot>"
 * @return esp_err_t
 */
esp_err_t console_start(const char *prompt);

/**
 * @brief 只初始化 Console，但不启动 REPL（可选）
 */
esp_err_t console_init_only(esp_console_repl_t **repl, const char *prompt);

#ifdef __cplusplus
}
#endif