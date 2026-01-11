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

typedef struct PIDParams {
    float P;
    float I;
    float D;
} PIDParams;
inline constexpr PIDParams PID_STB {0.018, 0.0, 0.0014};
inline constexpr PIDParams PID_VEL {0.7, 0.3, 0.0};
inline constexpr PIDParams PID_STEER {0.008, 0, 0.002};
// inline constexpr PIDParams PID_WHEELSPEED {0.07, (0.07f/50.0f), 0}; // ki = kp / 50.0f

constexpr float MOTOR_MAX_TORQUE = 45.0f;
constexpr int MOTOR_MAX_STEERING = 50;
enum SpeedGear {
    SLOW = 0,   // 慢走
    MEDIUM,     // 快走 / 慢跑
    FAST,       // 跑步
    MAX_SPEED
};

constexpr int BALANCE_PITCH_THRESHOLD = 60;
constexpr int BALANCE_WAITTING_TIME = 1000;
constexpr int BALANCE_ENABLE_STEERING_I_TIME = 2000;

enum BotState {
    BOT_UNINIT = 0,     // [生命周期] 刚上电，未调用 init
    BOT_READY,          // [生命周期] init完成，IMU更新中，电机未使能 (此时 PID Tuner 可见波形)
    BOT_FALL,           // [运行态] start完成，电机使能，但机器倒地 (PWM=0)
    BOT_WAIT_BALANCE,   // [运行态] 扶起中，等待进入平衡
    BOT_BALANCE         // [运行态] 正常平衡控制中
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
    int start();
    void stop();
    void attachImu(std::shared_ptr<Imu> imu) {
        imu_ = imu;
    }

    /* used for AI control */
    void move(direction_t dir, int distance_cm);
    void rotate(int angle);
    void turnAround(void);

    /* used for controller control */
    void setMotion(float speed, float steering);

    /* used for tuner */
    enum class PIDType { STB, VEL, STEER };
    IPID* getPID(PIDType type);
    PIDParams* getPIDParam(PIDType type);
    void updatePIDParam(PIDType type, float p, float i, float d);
    float getPitch() { return imu_->getPitch(); }
    float getMotorLeftSpeed() { return motor_l.shaft_velocity; }
    float getMotorRightSpeed() { return motor_r.shaft_velocity; }
    float getMidpoint() { return mid_value_; };
    void setMidpoint(float midpoint) {
        mid_value_ = midpoint;
    }
    void setSpeedGear(SpeedGear gear);
    float getMaxSpeedByGear() const;
private:
    Motor();
    void task();
    void resetAllPid();
    void checkBalanceStatus(float mpu_pitch);
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
    SpeedGear speed_gear_ = SpeedGear::SLOW;

    GyroPID pid_stb_;           // 直立环
    PIDController pid_vel_;     // 整车速度环
    GyroPID pid_steering_;      // 转向环
    // save tune pid value
    PIDParams tune_stb_;
    PIDParams tune_vel_;
    PIDParams tune_steering_;

    LowPassFilter lpf_throttle;
    LowPassFilter lpf_steering;

    std::shared_ptr<Imu> imu_;

    // timer for FOC loop
    esp_timer_handle_t foc_timer_;
    static constexpr uint32_t FOC_TIMER_PERIOD_US = 250; // foc_current选用4kHz控制频率
    static void IRAM_ATTR foc_timer_callback(void* arg);
    int start_foc_timer();

    volatile BotState bot_state_ = BOT_UNINIT;
};

#endif