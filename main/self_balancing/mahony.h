#pragma once
#include <math.h>

class Mahony {
public:
    Mahony();

    void setGains(float kp, float ki);
    void reset();

    // dt: 秒
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt);

    float getPitchRad();   // rad
    float getRoll();    // rad

private:
    // quaternion
    float q0, q1, q2, q3;

    // PI parameters
    float twoKp;
    float twoKi;

    // integral feedback
    float integralFBx;
    float integralFBy;
    float integralFBz;

    static float invSqrt(float x);
};