#include "settings.h"

#include <esp_log.h>
#include <nvs_flash.h>

#define TAG "Settings"

Settings::Settings(const std::string& ns, bool read_write) : ns_(ns), read_write_(read_write) {
    nvs_open(ns.c_str(), read_write_ ? NVS_READWRITE : NVS_READONLY, &nvs_handle_);
}

Settings::~Settings() {
    if (nvs_handle_ != 0) {
        if (read_write_ && dirty_) {
            ESP_ERROR_CHECK(nvs_commit(nvs_handle_));
        }
        nvs_close(nvs_handle_);
    }
}

std::string Settings::GetString(const std::string& key, const std::string& default_value) {
    if (nvs_handle_ == 0) {
        return default_value;
    }

    size_t length = 0;
    if (nvs_get_str(nvs_handle_, key.c_str(), nullptr, &length) != ESP_OK) {
        return default_value;
    }

    std::string value;
    value.resize(length);
    ESP_ERROR_CHECK(nvs_get_str(nvs_handle_, key.c_str(), value.data(), &length));
    while (!value.empty() && value.back() == '\0') {
        value.pop_back();
    }
    return value;
}

void Settings::SetString(const std::string& key, const std::string& value) {
    if (read_write_) {
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle_, key.c_str(), value.c_str()));
        dirty_ = true;
    } else {
        ESP_LOGW(TAG, "Namespace %s is not open for writing", ns_.c_str());
    }
}

int32_t Settings::GetInt(const std::string& key, int32_t default_value) {
    if (nvs_handle_ == 0) {
        return default_value;
    }

    int32_t value;
    if (nvs_get_i32(nvs_handle_, key.c_str(), &value) != ESP_OK) {
        return default_value;
    }
    return value;
}

void Settings::SetInt(const std::string& key, int32_t value) {
    if (read_write_) {
        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle_, key.c_str(), value));
        dirty_ = true;
    } else {
        ESP_LOGW(TAG, "Namespace %s is not open for writing", ns_.c_str());
    }
}

size_t Settings::getBytesLength(const std::string& key) {
    size_t len = 0;
    if (key.empty()) {
        return 0;
    }
    esp_err_t err = nvs_get_blob(nvs_handle_, key.c_str(), NULL, &len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_blob len fail: %s %s", key.c_str(), esp_err_to_name(err));
        return 0;
    }
    return len;
}

size_t Settings::getBytes(const std::string& key, void * buf, size_t maxLen) {
    size_t len = getBytesLength(key);
    if (!len || !buf || !maxLen) {
        return len;
    }
    if (len > maxLen) {
        ESP_LOGE(TAG, "not enough space in buffer: %u < %u", maxLen, len);
        return 0;
    }
    esp_err_t err = nvs_get_blob(nvs_handle_, key.c_str(), buf, &len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_blob fail: %s %s", key.c_str(), esp_err_to_name(err));
        return 0;
    }
    return len;
}

size_t Settings::putBytes(const std::string& key, const void* value, size_t len) {
    if (key.empty() || !value || !len) {
        return 0;
    }
    esp_err_t err = nvs_set_blob(nvs_handle_, key.c_str(), value, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob fail: %s %s", key.c_str(), esp_err_to_name(err));
        return 0;
    }
    return len;
}

float Settings::GetFloat(const std::string& key, float default_value) {
    float value = default_value;
    getBytes(key, (void*) &value, sizeof(float));
    return value;
}

void Settings::SetFloat(const std::string& key, float value) {
    if (read_write_) {
        putBytes(key, (void*)&value, sizeof(float));
        dirty_ = true;
    } else {
        ESP_LOGW(TAG, "Namespace %s is not open for writing", ns_.c_str());
    }
}

void Settings::EraseKey(const std::string& key) {
    if (read_write_) {
        auto ret = nvs_erase_key(nvs_handle_, key.c_str());
        if (ret != ESP_ERR_NVS_NOT_FOUND) {
            ESP_ERROR_CHECK(ret);
        }
    } else {
        ESP_LOGW(TAG, "Namespace %s is not open for writing", ns_.c_str());
    }
}

void Settings::EraseAll() {
    if (read_write_) {
        ESP_ERROR_CHECK(nvs_erase_all(nvs_handle_));
    } else {
        ESP_LOGW(TAG, "Namespace %s is not open for writing", ns_.c_str());
    }
}
