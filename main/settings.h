#ifndef SETTINGS_H
#define SETTINGS_H

#include <string>
#include <nvs_flash.h>
#include "string.h"

class Settings {
public:
    Settings(const std::string& ns, bool read_write = false);
    ~Settings();

    std::string GetString(const std::string& key, const std::string& default_value = "");
    void SetString(const std::string& key, const std::string& value);
    int32_t GetInt(const std::string& key, int32_t default_value = 0);
    void SetInt(const std::string& key, int32_t value);
    float GetFloat(const std::string& key, float default_value=0);
    void SetFloat(const std::string& key, float value); 
    void EraseKey(const std::string& key);
    void EraseAll();

private:
    size_t getBytesLength(const std::string& key);
    size_t getBytes(const std::string& key, void * buf, size_t maxLen);
    size_t putBytes(const std::string&, const void* value, size_t len);
    std::string ns_;
    nvs_handle_t nvs_handle_ = 0;
    bool read_write_ = false;
    bool dirty_ = false;
};

#endif
