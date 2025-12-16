#include "mahony.h"

Mahony::Mahony()
{
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    twoKp = 2.0f * 0.5f;
    twoKi = 2.0f * 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
}

void Mahony::setGains(float kp, float ki)
{
    twoKp = 2.0f * kp;
    twoKi = 2.0f * ki;
}

void Mahony::reset()
{
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
}

void Mahony::update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt)
{
    // deg/s → rad/s
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // accel valid?
    if (!(ax == 0 && ay == 0 && az == 0)) {
        // normalize acc
        float recip = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recip;
        ay *= recip;
        az *= recip;

        // estimated gravity direction
        float halfvx = q1*q3 - q0*q2;
        float halfvy = q0*q1 + q2*q3;
        float halfvz = q0*q0 - 0.5f + q3*q3;

        // error = cross product
        float halfex = (ay*halfvz - az*halfvy);
        float halfey = (az*halfvx - ax*halfvz);
        float halfez = (ax*halfvy - ay*halfvx);

        // integral feedback
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }

        // proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // integrate quaternion
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0;
    float qb = q1;
    float qc = q2;

    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    // normalize quaternion
    float recip = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recip;
    q1 *= recip;
    q2 *= recip;
    q3 *= recip;
}

float Mahony::getPitchRad()
{
    // rad
    return asinf(-2.0f * (q1*q3 - q0*q2));
}

float Mahony::getRoll()
{
    return atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
}

float Mahony::invSqrt(float x)
{
	float halfx = 0.5f * x;
	union { float f; long l; } i;
	i.f = x;
	i.l = 0x5f3759df - (i.l >> 1);
	float y = i.f;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}
