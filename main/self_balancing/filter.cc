#include "filter.h"
#include "esp_log.h"
#include "esp_timer.h"

Filter::Filter(FilterType type) : type_(type) {}

void Filter::reset() {
    kalman_.setAngle(pitch_acc_);
    pitch_comp_ = pitch_gyro_ = pitch_kalman_ = pitch_acc_;
}

esp_err_t Filter::update(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ) {
    long now = esp_timer_get_time() / 1000; // ms

    pitch_acc_ = atan2(accY, accZ + fabsf(accX)) * RAD_TO_DEG;

    if (timer_ == 0) {
        // 第一次调用：仅初始化，不滤波
        reset();
        angle_.pitch = pitch_acc_;
        timer_ = now;
    } else {
        float dt = (now - timer_) / 1000.0f;
        timer_ = now;

        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
        if ((pitch_acc_ < -90 && pitch_kalman_ > 90) || (pitch_acc_ > 90 && pitch_kalman_ < -90)) {
            reset();
        }

        switch (type_) {
            case FilterType::COMPLEMENTARY:
                pitch_comp_ = ALPHA * (pitch_comp_ + gyrX * dt) + (1 - ALPHA) * pitch_acc_;
                angle_.pitch = pitch_comp_;
                break;
            case FilterType::KALMAN:
                pitch_kalman_ = kalman_.getAngle(pitch_acc_, gyrX, dt);
                angle_.pitch = pitch_kalman_;
                break;
            case FilterType::GYRO_ONLY:
                pitch_gyro_ += gyrX * dt;
                angle_.pitch = pitch_gyro_;
                break;
            default:
                angle_.pitch = pitch_acc_;
                break;
        }

        lowPass(gyrX, gyroX_fv_);
        angle_.yaw += lowPass(gyrZ, gyroZ_fv_) * dt;
    }

    return ESP_OK;
}

float Filter::lowPass(float input, float& output, float alpha) {
    output = alpha * input + (1 - alpha) * output;
    return output;
}