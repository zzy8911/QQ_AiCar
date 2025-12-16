#pragma once
#include "esp_err.h"
#include "kalman.h"
#include "mahony.h"
#include <cmath>

enum class FilterType {
    NONE,
    GYRO_ONLY,
    COMPLEMENTARY,
    KALMAN,
    MAHONY
};

enum class CoordinateSystem {
    X_FORWARD, // 俯仰的时候X轴数值变化，也就意味着沿着6轴的X轴加速度方向前进后退
    Y_FORWARD  // 俯仰的时候Y轴数值变化
};

struct Attitude {
    float pitch;
    float roll;
    float yaw;
};

class Filter {
public:
    Filter(FilterType type = FilterType::NONE, CoordinateSystem coord_sys = CoordinateSystem::X_FORWARD);
    esp_err_t update(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ);
    void reset();

    float getPitch() { return angle_.pitch; };
	float getRoll() { return angle_.roll; };
	float getYaw() { return angle_.yaw; };

    float lowPassGyroPitch() { return gyroPitch_fv_; };
    float lowPassGyroZ() { return gyroZ_fv_; };

private:
    float lowPass(float input, float& output, float alpha = 0.2f);

    FilterType type_;
    CoordinateSystem coord_sys_;
    Kalman kalman_;
    Mahony mahony_;
    Attitude angle_{0.0f, 0.0f, 0.0f};

    float pitch_acc_ = 0.0f; // lowpass filter
    float pitch_kalman_ = 0.0f;
    float pitch_comp_ = 0.0f;
    float pitch_gyro_ = 0.0f;
    long timer_ = 0;

    float gyroPitch_fv_ = 0.0f;
    float gyroZ_fv_ = 0.0f;

    static constexpr float ALPHA = 0.95f;
    static constexpr float RAD_TO_DEG = 57.29577951f; /*!< Radians to degrees: 360 / 2.0 / PI */
};
