#ifndef __SIMPLEFOCDEBUG_H__
#define __SIMPLEFOCDEBUG_H__

#include "esp_log.h"

#ifndef SIMPLEFOC_DISABLE_DEBUG

class SimpleFOCDebug {
public:
    static void enable(void *port = nullptr) {
        // IDF 不需要初始化
    }

    // 统一入口由宏处理，不需要多重重载
    static void log(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        esp_log_writev(ESP_LOG_INFO, TAG, fmt, args);
        va_end(args);

        esp_log_write(ESP_LOG_INFO, TAG, "\n");
    }

private:
    static inline constexpr const char* TAG = "simplefoc";
};


// ====== 替代原 Arduino 方式的 DEBUG 宏 ======
#define SIMPLEFOC_DEBUG(msg, ...) \
    SimpleFOCDebug::log(msg, ##__VA_ARGS__)



#else   // SIMPLEFOC_DISABLE_DEBUG

// debug 完全关闭
#define SIMPLEFOC_DEBUG(msg, ...)

#endif  // SIMPLEFOC_DISABLE_DEBUG

#endif // __SIMPLEFOCDEBUG_H__