#include "filter.h"
#include "esp_log.h"
#include "esp_timer.h"

Filter::Filter(FilterType type, CoordinateSystem coord_sys) : type_(type), coord_sys_(coord_sys){}

void Filter::reset() {
    kalman_.setAngle(pitch_acc_);
    mahony_.reset();
    pitch_comp_ = pitch_gyro_ = pitch_kalman_ = pitch_acc_;
}

esp_err_t Filter::update(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ) {
    unsigned long now = esp_timer_get_time();
    float gyro_pitch = 0;

    // MAHONY 滤波器不需要 pitch_acc_，因此当滤波器类型为 MAHONY 时，可以跳过昂贵的 atan2f 和 sqrtf 计算。
    if (timer_ == 0 || type_ != FilterType::MAHONY) {
        if (coord_sys_ == CoordinateSystem::X_FORWARD) {
            pitch_acc_ = atan2f(
                accX,
                sqrtf(accY * accY + accZ * accZ)
            ) * RAD_TO_DEG;
        } else if (coord_sys_ == CoordinateSystem::Y_FORWARD) {
            pitch_acc_ = atan2f(
                accY,
                sqrtf(accX * accX + accZ * accZ)
            ) * RAD_TO_DEG;
        }
    }

    // 根据坐标系确定用于 Pitch 的角速度
    if (coord_sys_ == CoordinateSystem::X_FORWARD) {
        gyro_pitch = gyrY;
    } else if (coord_sys_ == CoordinateSystem::Y_FORWARD) {
        gyro_pitch = gyrX;
    }
    // ------------------------------------------

    if (timer_ == 0) {
        // 第一次调用：仅初始化，不滤波
        reset();
        angle_.pitch = pitch_acc_;
        timer_ = now;
    } else {
        float dt = (now - timer_) * 1e-6f;
        timer_ = now;

        // 仅对使用 pitch_acc_ 的滤波器执行跳变修正（Complementary/Kalman）
        if (type_ != FilterType::MAHONY) {
            // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
            // 注意：这里仍然是与 pitch_kalman_ 比较，如果需要支持 comp 滤波器，应该使用 pitch_comp_
            if ((pitch_acc_ < -90 && pitch_kalman_ > 90) || (pitch_acc_ > 90 && pitch_kalman_ < -90)) {
                reset();
            }
        }

        switch (type_) {
            case FilterType::COMPLEMENTARY:
                pitch_comp_ = ALPHA * (pitch_comp_ + gyro_pitch * dt) + (1.0f - ALPHA) * pitch_acc_;
                angle_.pitch = pitch_comp_;
                break;
            case FilterType::KALMAN:
                pitch_kalman_ = kalman_.getAngle(pitch_acc_, gyro_pitch, dt);
                angle_.pitch = pitch_kalman_;
                break;
            case FilterType::GYRO_ONLY:
                pitch_gyro_ += gyro_pitch * dt;
                angle_.pitch = pitch_gyro_;
                break;
            case FilterType::MAHONY:
                mahony_.update(
                    gyrX, gyrY, gyrZ,
                    accX, accY, accZ,
                    dt
                );
                angle_.pitch = -mahony_.getPitchRad() * RAD_TO_DEG;
                break;
            default:
                // 如果没有定义其他滤波器，则使用加速度计值（需要 pitch_acc_）
                angle_.pitch = pitch_acc_;
                break;
        }

        lowPass(gyro_pitch, gyroPitch_fv_);
        angle_.yaw += lowPass(gyrZ, gyroZ_fv_) * dt;
    }

    return ESP_OK;
}

float Filter::lowPass(float input, float& output, float alpha) {
    output = alpha * input + (1 - alpha) * output;
    return output;
}