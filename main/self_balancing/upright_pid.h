#pragma once
#include <algorithm>

class UprightPID {
public:
    UprightPID(float kp, float ki, float kd, float limit)
        : kp_(kp), ki_(ki), kd_(kd)
    {
        kpMin_ = -limit;
        kpMax_ = limit;
        kiMin_ = -limit / 2.0f;
        kiMax_ = limit / 2.0f;
        kdMin_ = -limit / 2.0f;
        kdMax_ = limit / 2.0f;
        outMin_ = -limit;
        outMax_ = limit;
    }
    float operator()(float targetAngleX, float angleX, float gyroX)
    {
        float error = angleX + targetAngleX;

        float pOut = kp_ * error;
        kiOut_ += ki_ * error;
        float dOut = kd_ * gyroX;

        // 限制各部分输出范围
        pOut = std::clamp(pOut, kpMin_, kpMax_);
        kiOut_ = std::clamp(kiOut_, kiMin_, kiMax_);
        dOut = std::clamp(dOut, kdMin_, kdMax_);

        pidOut_ = pOut + kiOut_ + dOut;
        pidOut_ = std::clamp(pidOut_, outMin_, outMax_);

        return pidOut_;
    }
    void reset()
    {
        kiOut_ = 0.0f;
        pidOut_ = 0.0f;
    }

private:
    float kp_, ki_, kd_;
    float kpMin_, kpMax_;
    float kiMin_, kiMax_;
    float kdMin_, kdMax_;
    float outMin_, outMax_;

    float kiOut_ = 0.0f;
    float pidOut_ = 0.0f;
};
