#pragma once
#include "esp_err.h"
#include "kalman.h"
#include <cmath>

enum class FilterType {
    NONE,
    GYRO_ONLY,
    COMPLEMENTARY,
    KALMAN
};

struct Attitude {
    float pitch;
    float roll;
    float yaw;
};

class Filter {
public:
    Filter(FilterType type = FilterType::NONE);
    esp_err_t update(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ);
    void reset();

    float getPitch() { return angle_.pitch; };
	float getRoll() { return angle_.roll; };
	float getYaw() { return angle_.yaw; };

    float lowPassGyroX() { return gyroX_fv_; };
    float lowPassGyroZ() { return gyroZ_fv_; };

private:
    float lowPass(float input, float& output, float alpha = 0.2f);

    FilterType type_;
    Kalman kalman_;
    Attitude angle_{0.0f, 0.0f, 0.0f};

    float pitch_acc_ = 0.0f;
    float pitch_kalman_ = 0.0f;
    float pitch_comp_ = 0.0f;
    float pitch_gyro_ = 0.0f;
    long timer_ = 0;

    float gyroX_fv_ = 0.0f;
    float gyroZ_fv_ = 0.0f;

    static constexpr float ALPHA = 0.93f;
    static constexpr float RAD_TO_DEG = 57.29577951f; /*!< Radians to degrees: 360 / 2.0 / PI */
};
