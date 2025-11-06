#include "motor.h"
#include <math.h>

#define _PI       3.14159265359f
#define _PI_2     1.57079632679f
#define _PI_3     1.0471975512f
#define _3PI_2    4.71238898038f
#define _SQRT3    1.73205080757f
#define _1_SQRT3  0.57735026919f
#define _2_SQRT3  1.15470053838f

#define TAG     "MOTOR"

static float normalize_angle(float angle) {
    float a = fmodf(angle, 2 * _PI);
    return a >= 0 ? a : (a + 2 * _PI);
}

static float cal_Iq_Id(float current_a, float current_b, float angle_el) {
    float I_alpha = current_a;
    float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

    float ct = cos(angle_el);
    float st = sin(angle_el);
    //float I_d = I_alpha * ct + I_beta * st;
    float I_q = I_beta * ct - I_alpha * st;

    return I_q;
}

Motor::Motor(int pp, int dir, float vbus,
             BLDCDriver3PWM& driver,
             MT6701& encoder,
             CurrSense& current_sensor)
    : pp_(pp),
      dir_(dir),
      vbus_(vbus),
      driver_(driver),
      encoder_(encoder),
      current_sensor_(current_sensor),
      zero_electrical_angle_(0.0f),
      vel_filter_(0.01f),
      curr_filter_(0.05f),
      current_pid_{0.5f, 0.2f, 0.0f, 100000.0f, vbus / 2},
      vel_pid_{2.0f, 0.0f, 0.0f, 100000.0f, vbus / 2},
      angle_pid_{2.0f, 0.0f, 0.0f, 100000.0f, 100.0f}
{
}

void Motor::init() {
}

bool Motor::alignSensor() {
    ESP_LOGI(TAG, "Align sensor.");

    // if unknown natural direction
    if (dir_ == Direction::UNKNOWN) {
        // find natural direction
        // move one electrical revolution forward
        for (int i = 0; i <=500; i++ ) {
            float angle = _3PI_2 + _2PI * i / 500.0f;
            setTorqueSvpwm(voltage_sensor_align, angle);
            encoder_.update();
            _delay(2);
        }
        // take and angle in the middle
        encoder_.update();
        float mid_angle = encoder_.getAngle();
        // move one electrical revolution backwards
        for (int i = 500; i >=0; i-- ) {
            float angle = _3PI_2 + _2PI * i / 500.0f ;
            setTorqueSvpwm(voltage_sensor_align, angle);
            encoder_.update();
            _delay(2);
        }
        encoder_.update();
        float end_angle = encoder_.getAngle();
        setTorqueSvpwm(0, 0);
        _delay(200);

        // determine the direction the sensor moved
        float moved = fabs(mid_angle - end_angle);
        if (moved<MIN_ANGLE_DETECT_MOVEMENT) { // minimum angle to detect movement
            ESP_LOGI(TAG, "Failed to notice movement");
            return 0; // failed calibration
        } else if (mid_angle < end_angle) {
            ESP_LOGI(TAG, "sensor_direction==CCW");
            dir_ = Direction::CCW;
        } else{
            ESP_LOGI(TAG, "sensor_direction==CW");
            dir_ = Direction::CW;
        }
        // check pole pair number
        if ( fabs(moved*pp_ - _2PI) > 0.5f ) { // 0.5f is arbitrary number it can be lower or higher!
            ESP_LOGI(TAG, "PP check: fail - estimated pp: %f", _2PI/moved);
        } else 
            ESP_LOGI(TAG, "PP check: OK!");
    } else ESP_LOGI(TAG, "Skip dir calib.");

    // zero electric angle not known
    if (zero_electrical_angle_ == 0.0f) {
        // align the electrical phases of the motor and sensor
        // set angle -90(270 = 3PI/2) degrees
        setTorqueSvpwm(voltage_sensor_align, _3PI_2);
        _delay(700);
        // read the sensor
        encoder_.update();
        // get the current zero electric angle
        zero_electrical_angle_ = electricalAngle();
        //zero_electric_angle =  _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
        _delay(20);
        ESP_LOGI(TAG, "Zero elec. angle: %f", zero_electrical_angle_);

        // stop everything
        setTorqueSvpwm(0, 0);
        _delay(200);
    } else ESP_LOGI(TAG, "Skip offset calib.");

    return 0;

#if 1
    setTorqueSvpwm(2.0f, _3PI_2);
    vTaskDelay(pdMS_TO_TICKS(500));
    encoder_.update();
    zero_electrical_angle_ = electricalAngle();
    setTorqueSvpwm(0.0f, 0.0f);
    ESP_LOGI(TAG, "Zero electrical angle: %f rad", zero_electrical_angle_);
#else
    const float SAFE_UQ = 0.5f; // 0.5V 作为示例（可改）
    const int DURATION_MS = 300; // 300 ms 对齐时间
    const float MAX_SAFE_CURRENT = 5.0f; // 根据你的硬件设定阈值

    setTorqueSvpwm(SAFE_UQ, _3PI_2);

    int elapsed = 0;
    const int step = 20;
    while (elapsed < DURATION_MS) {
        vTaskDelay(pdMS_TO_TICKS(step));
        elapsed += step;
        update();

        float i = getCurrent();
        ESP_LOGI(TAG, "Aligning... Current: %.2f A", i);
        if (i > MAX_SAFE_CURRENT) {
            setTorqueSvpwm(0.0f, 0.0f);
            ESP_LOGW(TAG, "align aborted: over current %.2f A", i);
            return -1;
        }
    }
    zero_electrical_angle_ = electricalAngle();
    setTorqueSvpwm(0.0f, 0.0f);
    ESP_LOGI(TAG, "Zero electrical angle: %f rad", zero_electrical_angle_);
#endif
    return 0;
}

float Motor::electricalAngle() {
    float mech_angle = encoder_.getMechanicalAngle();
    return normalize_angle((float)(dir_ * pp_) * mech_angle - zero_electrical_angle_);
}

void Motor::setTorqueSvpwm(float uq, float angle_el) {
    if (uq < 0) angle_el += _PI;
    uq = fabsf(uq);
    angle_el = normalize_angle(angle_el + _PI_2);
    int sector = (int)floorf(angle_el / _PI_3) + 1;

    float T1 = _SQRT3 * sinf(sector * _PI_3 - angle_el) * uq / vbus_;
    float T2 = _SQRT3 * sinf(angle_el - (sector - 1.0f) * _PI_3) * uq / vbus_;
    float T0 = 1.0f - T1 - T2;

    float Ta = 0, Tb = 0, Tc = 0;
    switch (sector) {
        case 1: Ta = T1+T2+T0/2; Tb = T2+T0/2; Tc = T0/2; break;
        case 2: Ta = T1+T0/2; Tb = T1+T2+T0/2; Tc = T0/2; break;
        case 3: Ta = T0/2; Tb = T1+T2+T0/2; Tc = T2+T0/2; break;
        case 4: Ta = T0/2; Tb = T1+T0/2; Tc = T1+T2+T0/2; break;
        case 5: Ta = T2+T0/2; Tb = T0/2; Tc = T1+T2+T0/2; break;
        case 6: Ta = T1+T2+T0/2; Tb = T0/2; Tc = T1+T0/2; break;
        default: break;
    }

    driver_.setPwm(Ta * vbus_, Tb * vbus_, Tc * vbus_);
}

float Motor::getAngle() {
    return dir_ * encoder_.getAngle();
}

float Motor::getVelocity() {
    return vel_filter_(dir_ * encoder_.getVelocity());
}

float Motor::getCurrent() {
    float iq = curr_filter_(cal_Iq_Id(current_sensor_.getCurrentA(),
                                  current_sensor_.getCurrentB(),
                                  electricalAngle()));
    ESP_LOGI(TAG, "Iq: %.2f A", iq);
    return iq;
}

void Motor::setTorque(float current_target) {
    float error = current_target - getCurrent();
    float uq = current_pid_(error);
    setTorqueSvpwm(uq, electricalAngle());
}

void Motor::setVelocity(float vel_target) {
    float error = vel_target - getVelocity();
    float current_target = vel_pid_(error * 180.0f / _PI);
    setTorque(current_target);
}

void Motor::setPosition(float angle_target) {
    float angle_actual = getAngle();
    float error = angle_target - angle_actual;
    float vel_target = angle_pid_(error);
    setVelocity(vel_target);
}

void Motor::update() {
    encoder_.update();
    current_sensor_.updatePhaseCurrents();
}
