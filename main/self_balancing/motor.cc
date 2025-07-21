#include "motor.h"
#include <memory>
#include <algorithm>

#define TAG "Motor"

Motor::Motor()
    : motor_l(7), motor_r(7),
      driver_l(MO0_1, MO0_2, MO0_3), driver_r(MO1_1, MO1_2, MO1_3),
      sensor_l(ENCODER_I2C_BUS),
      sensor_r(IMU_ENCODER_I2C_BUS),
      settings("motor", true),
      pid_stb_(PID_STB.P, PID_STB.I, PID_STB.D, MOTOR_MAX_TORQUE),
      pid_vel_(PID_VEL.P, PID_VEL.I, PID_VEL.D, 100000, MOTOR_MAX_TORQUE),
      pid_vel_tmp_(PID_VEL.P, PID_VEL.I, PID_VEL.D, 100000, MOTOR_MAX_TORQUE),
      pid_steering_(0.01, 0, 0.001, MOTOR_MAX_TORQUE / 2),
      lpf_throttle(0.5),
      lpf_steering(0.5) {}

Motor::~Motor() {
    if (motor_task_handle_ != nullptr) {
        vTaskDelete(motor_task_handle_);
        motor_task_handle_ = nullptr;
    }
    if (motor_task_stack_ != nullptr) {
        heap_caps_free(motor_task_stack_);
        motor_task_stack_ = nullptr;
    }
    sensor_l.deinit();
    sensor_r.deinit();
}

template <typename SensorType>
static void init_motor(BLDCMotor *motor, BLDCDriver3PWM *driver, SensorType *sensor)
{
    sensor->init();
    //连接motor对象与传感器对象
    motor->linkSensor(sensor);
    // PWM 频率 [Hz]
    driver->pwm_frequency = 20000;
    //供电电压设置 [V]
    driver->voltage_power_supply = 8;
    driver->init();
    motor->linkDriver(driver);
    //FOC模型选择
    motor->foc_modulation = FOCModulationType::SpaceVectorPWM;
    // motor.modulation_centered = 1.0;
    //运动控制模式设置
    motor->torque_controller = TorqueControlType::voltage;
    motor->controller = MotionControlType::torque;

    //速度低通滤波时间常数
    motor->LPF_velocity.Tf = 0.02f;
    motor->PID_velocity.output_ramp = 1000;
    // motor->PID_velocity.limit = MOTOR_MAX_SPEED; // rad/s
    //最大电机限制电机
    motor->voltage_limit = 8;

    //设置最大速度限制
    // motor->velocity_limit = MOTOR_MAX_SPEED;

#ifdef XK_WIRELESS_PARAMETER
    motor->useMonitoring(HAL::get_wl_tuning());
#else
    // Serial.begin(115200);
    // motor->monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
    // motor->useMonitoring(Serial);
    // motor->monitor_downsample = 100;  // disable monitor at first - optional
#endif
    //初始化电机
    motor->init();
    // motor->initFOC();
}

static void initFOC(BLDCMotor &motor, float offset)
{
    if (offset > 0) {
        ESP_LOGI(TAG, "has a offset value %.2f.", offset);
        Direction foc_direction = Direction::CW;
        motor.initFOC(offset, foc_direction);
    } else {
        if (motor.initFOC()) {
            ESP_LOGI(TAG, "motor zero electric angle: %.2f", motor.zero_electric_angle);
        }
    }
}

int Motor::init(void)
{
    ESP_LOGI(TAG, "Motor starting...");

    init_motor(&motor_l, &driver_l, &sensor_l);
    init_motor(&motor_r, &driver_r, &sensor_r);
    vTaskDelay(100);

    float l_offset = settings.GetFloat("l_offset", 0);
    float r_offset = settings.GetFloat("r_offset", 0);
    if (l_offset != 0 || r_offset != 0) {
        ESP_LOGI(TAG, "[motor]: set offset %f, %f", l_offset, r_offset);
        initFOC(motor_l, l_offset);
        initFOC(motor_r, r_offset);
    } else {
        ESP_LOGI(TAG, "motor: get config failed, try auto calibration.");

        initFOC(motor_l, 0);
        initFOC(motor_r, 0);

        settings.SetFloat("l_offset", motor_l.zero_electric_angle); // 0.9157872
        settings.SetFloat("r_offset", motor_r.zero_electric_angle); // 0.6583844
    }

    ESP_LOGI(TAG, "Motor ready.");

    motor_task_stack_ = (StackType_t*) heap_caps_malloc(4096 * sizeof(StackType_t), MALLOC_CAP_SPIRAM);
    if (motor_task_handle_ == nullptr) {
        motor_task_handle_ = xTaskCreateStaticPinnedToCore([](void* arg) {
                Motor* motor = (Motor*)arg;
                motor->task();
                vTaskDelete(NULL);
            },
            "MotorThread",
            4096,
            this,
            10,
            motor_task_stack_,
            &motor_task_tcb_,
            1); // Motor: CORE 1
        if (motor_task_handle_ == nullptr) {
            ESP_LOGE(TAG, "start motor_run task failed.");
            return -1;
        }
    }

    return 0;
}

static int taskModeUpdate(int &mode, bool &is_changed)
{
    int mpu_pitch = (int)(HAL::imu_get_pitch());
    // ESP_LOGI(TAG, "pitch: %d", mpu_pitch);
    static int last_mode = BOT_RUNNING_MODE;
    static unsigned long last_change_time = 0;
    static bool is_timing = false;
    int mode_tmp = BOT_RUNNING_MODE;

    if (abs(mpu_pitch) < 60) {
        mode_tmp = BOT_RUNNING_BALANCE;
    }

    if (mode_tmp != mode && !is_timing) {
        last_change_time = millis(); // 记录状态变化的时间
        is_timing = true;
    }
    if (mode_tmp == mode) {
        is_timing = false;
    }

    if (is_timing && millis() - last_change_time >= 1000) {
        mode = mode_tmp; // 更新模式
        is_changed = true; // 标记状态变化
        ESP_LOGE(TAG, "pitch: %d mode from %d change to %d", mpu_pitch, last_mode, mode);
        last_mode = mode; // 更新上一次模式
    }
    
    return 0;
}

void Motor::resetAllPid()
{
    pid_stb_.reset();
    pid_vel_.reset();
    pid_steering_.reset();
}

int Motor::checkBalanceStatus(float mpu_pitch)
{
    static unsigned long start_wait = 0;
    
    if (abs(mpu_pitch - mid_value_) > BALANCE_STOP_PITCH_OFFSET) {
        balance_status_ = BALANCE_OFF;
        return -1;
    }

    if (balance_status_ == BALANCE_RUNNING) {
        return 0;
    }

    if (balance_status_ == BALANCE_OFF) {
        balance_status_ = BALANCE_WATTING;
        start_wait = millis();
        return -1;
    }

    if (balance_status_ == BALANCE_WATTING) {
        if (millis() < start_wait + BALANCE_WAITTING_TIME) {
            return -1;
        }
    }
    resetAllPid();
    balance_status_ = BALANCE_RUNNING;
    return 0;
}

int Motor::runBalanceTask()
{
    // float voltage_control;
    static unsigned long ctlr_start_ms = 0; 
    int rc = 0;
    float speed = 0;
    static size_t count = 0;

    float mpu_pitch = HAL::imu_get_pitch();
    // ESP_LOGI(TAG, "mpu_pitch: %.2f, throttle_: %.2f, steering_: %.2f", mpu_pitch, throttle_, steering_);

    rc = checkBalanceStatus(mpu_pitch);
    if (rc) {
        motor_l.target = 0;
        motor_r.target = 0;
        goto out;
    }

    /* Parallel PID */
    stb_adj_ = pid_stb_(mid_value_, mpu_pitch, HAL::lowPassGyroX());

    /* every 4th loop, run speed and steering PID */
    if (count % 4 == 0) {
        // speed
        if (throttle_ != 0) {
            pid_vel_.I = 0;
            ctlr_start_ms = millis();
        } else {
            if (millis() > ctlr_start_ms + BALANCE_ENABLE_STEERING_I_TIME) {
                pid_vel_.I = pid_vel_tmp_.I;
            }
        }
        // When rotating in the same direction, one has a positive sign and the other negative, so the speeds are subtracted.
        speed = (motor_l.shaft_velocity - motor_r.shaft_velocity) / 2.0f;
        speed_adj_ = pid_vel_(lpf_throttle(throttle_) - speed);

        // steering
        steering_adj_ = pid_steering_(lpf_steering(steering_), 0.0f, HAL::lowPassGyroZ());
    }

    motor_l.target = -(stb_adj_ + speed_adj_ + steering_adj_);
    motor_r.target = (stb_adj_ + speed_adj_ - steering_adj_);

    count++;
out:
    motor_l.move();
    motor_r.move();
    return rc;
}

void Motor::task()
{
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int motor_task = BOT_RUNNING_MODE;
    bool is_task_changed = false;

    // Execute every 6 milliseconds
    while(1) {
        is_task_changed = false;

        HAL::imu_update(); // 更新IMU数据

        motor_l.loopFOC();
        motor_r.loopFOC();

        taskModeUpdate(motor_task, is_task_changed);
        switch(motor_task) {
        case BOT_RUNNING_MODE:
            motor_l.move(0);
            motor_r.move(0);
            break;
        case BOT_RUNNING_BALANCE:
            runBalanceTask();
            break;
        default:
            break;
        }

        // motor_0.monitor();
        // motor_1.monitor();
        // Serial.println(motor_config[id].position);
        // vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void Motor::setSpeed(float speed, float steering)
{
    if (throttle_ != speed || steering_ != steering) {
        // ESP_LOGE(TAG, "speed: %d steering %d.", speed, steering);

        speed = std::clamp(speed, -static_cast<float>(MOTOR_MAX_SPEED), static_cast<float>(MOTOR_MAX_SPEED));
        steering = std::clamp(steering, -static_cast<float>(MOTOR_MAX_STEERING), static_cast<float>(MOTOR_MAX_STEERING));

        throttle_ = speed;
        steering_ = steering;
        // ESP_LOGE(TAG, "throttle: %.2f steering %.2f.", throttle_, steering_);
    }
}

void Motor::move(direction_t dir, int distance_cm) {
    float distance_m = static_cast<float>(distance_cm) / 100.0f;
    float move_time_sec = distance_m / TARGET_SPEED_MPS;  // t = s / v

    // 开始行驶
    if (dir == FORWARD) {
        setSpeed(-TARGET_SPEED_RADS, 0);
    } else if (dir == BACKWARD) {
        setSpeed(TARGET_SPEED_RADS, 0);
    } else {
        ESP_LOGE(TAG, "Invalid direction for motor move.");
        return;
    }

    // 延时一段时间，表示运行这么久就可以了
    vTaskDelay(pdMS_TO_TICKS(move_time_sec * 1000));

    // 停车
    setSpeed(0.0f, 0.0f);
}

void Motor::turnAround(void) {
    setSpeed(0, 30);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setSpeed(0.0f, 0.0f);
}