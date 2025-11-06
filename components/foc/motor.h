// main/motor.h
#pragma once

#include "mt6701.h"
#include "inline_current.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "esp_log.h"
#include "esp_hal_bldc_3pwm.h"

enum Direction : int8_t {
    CW      = 1,  // clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   // not yet known or invalid state
};

class Motor {
public:
    // 构造函数
    Motor(int pp, int dir, float vbus,
          BLDCDriver3PWM& driver,
          MT6701& encoder,
          CurrSense& current_sensor);

    void init();
    bool alignSensor();
    void setTorque(float current);
    void setVelocity(float vel);
    void setPosition(float pos);
    void update();

    float getAngle();
    float getVelocity();
    float getCurrent();

    void setTorqueSvpwm(float uq, float angle_el);

    float voltage_sensor_align;

private:
    // 电机参数
    int pp_;
    int dir_ = UNKNOWN;
    float vbus_;

    // 硬件接口
    BLDCDriver3PWM& driver_;
    MT6701& encoder_;
    CurrSense& current_sensor_;

    float zero_electrical_angle_;

    // 滤波器
    LowPassFilter vel_filter_, curr_filter_;

    // PID 控制器
    PIDController current_pid_, vel_pid_, angle_pid_;

    // 内部函数
    float electricalAngle();
    
};
