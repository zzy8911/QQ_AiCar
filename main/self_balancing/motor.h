#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "esp_log.h"
#include "boards/self-balancing-robot/config.h"
#include "driver/spi_master.h"
#include "settings.h"
#include <SimpleFOC.h>
#include "gyro_pid.h"
#include <memory>
#include "imu.h"

struct PIDParams {
    float P;
    float I;
    float D;
};
inline constexpr PIDParams PID_STB {0.01f, 0.0f, 0.003};
inline constexpr PIDParams PID_VEL {0.061f, 0.0061f, 0.0f};

constexpr float MOTOR_MAX_TORQUE = 45.0f;
constexpr int MOTOR_MAX_SPEED = 20;
constexpr int MOTOR_MAX_STEERING = 50;

constexpr int BALANCE_PITCH_THRESHOLD = 60;
constexpr int BALANCE_WAITTING_TIME = 1000;
constexpr int BALANCE_ENABLE_STEERING_I_TIME = 3000;

enum BOT_STATUS {
    BOT_FALL = 0,
    BOT_WAIT_BALANCE = 1,
    BOT_BALANCE = 2,
};

constexpr float WHEEL_RADIUS_M = 0.0325f;  // 65mm 轮子
constexpr float TARGET_SPEED_MPS = 0.3f;   // 目标线速度：0.3 m/s
constexpr float TARGET_SPEED_RADS = TARGET_SPEED_MPS / WHEEL_RADIUS_M;  // ≈9.23 rad/s

class Motor {
public:
    enum direction_t {
        FORWARD = 0,
        BACKWARD
    };

    static Motor& getInstance() {
        static Motor instance;
        return instance;
    };
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    ~Motor();

    int init();
    void setMotion(float speed, float steering);
    void move(direction_t dir, int distance_cm);
    void rotate(int angle);
    void turnAround(void);

    void adjustMidValue(float delta) {
        mid_value_ += delta;
    }
    float getMidValue() {
        return mid_value_;
    }

    void attachImu(std::shared_ptr<Imu> imu) {
        imu_ = imu;
    }
private:
    Motor();
    void task();
    void resetAllPid();
    void checkBalanceStatus(float mpu_pitch, BOT_STATUS &bot_state);
    int runBalanceTask();

    // Motor objects
    BLDCMotor motor_l;
    BLDCMotor motor_r;
    BLDCDriver3PWM driver_l;
    BLDCDriver3PWM driver_r;
    // Encoder sensors
    MT6701 sensor_l;
    MT6701 sensor_r;
    // current sense
    InlineCurrentSense cs_l;
    InlineCurrentSense cs_r;

    StackType_t* motor_task_stack_ = nullptr;
    StaticTask_t motor_task_tcb_;
    TaskHandle_t motor_task_handle_ = nullptr;

    Settings settings;

    float mid_value_ = 0.5f; // 偏置参数
    float throttle_ = 0;
    float steering_ = 0;

    GyroPID pid_stb_;
    PIDController pid_vel_;
    PIDController pid_vel_tmp_;
    GyroPID pid_steering_;
    float stb_adj_ = 0;
    float speed_adj_  = 0;
    float steering_adj_ = 0;

    LowPassFilter lpf_throttle;
    LowPassFilter lpf_steering;

    std::shared_ptr<Imu> imu_;

    // timer for FOC loop
    esp_timer_handle_t foc_timer_;
    static constexpr uint32_t FOC_TIMER_PERIOD_US = 500; // 选用2kHz控制频率, 4Khz对系统实时性要求比较高，比较吃力了
    static void IRAM_ATTR foc_timer_callback(void* arg);
    SemaphoreHandle_t foc_sem_ = nullptr;
    int start_foc_timer();
};

#endif