#include "motor.h"
#include <memory>
#include <algorithm>
#include "pid_tuner.h"
#include "application.h"
#include "assets/lang_config.h"

#define TAG "Motor"

static constexpr float SPEED_GEAR_MAX[] = {
    40.0f,   // ≈ 1.30 m/s
    60.0f,   // ≈ 1.95 m/s
    80.0f    // ≈ 2.60 m/s
};

/*
 * 控制逻辑
直立 + 速度 + 转向（并行）
       ↓
    目标轮速
       ↓
    电流 PID
 */

Motor::Motor()
    : motor_l(7), motor_r(7),
      driver_l(MO0_1, MO0_2, MO0_3), driver_r(MO1_1, MO1_2, MO1_3),
      sensor_l(SPI2_HOST, ENCODER_SCK, ENCODER_MISO, GPIO_NUM_NC, ENCODER_CS0),
      sensor_r(SPI2_HOST, ENCODER_SCK, ENCODER_MISO, GPIO_NUM_NC, ENCODER_CS1),
      cs_l(SAMPLE_RESISTOR_VALUE, SAMPLE_AMPLIFIER_GAIN, CS0_PHASE_A, CS0_PHASE_B, NOT_SET),
      cs_r(SAMPLE_RESISTOR_VALUE, SAMPLE_AMPLIFIER_GAIN, CS1_PHASE_A, CS1_PHASE_B, NOT_SET),
      settings("motor", true),
      pid_stb_(PID_STB.P, PID_STB.I, PID_STB.D, MOTOR_MAX_TORQUE),
      pid_vel_(PID_VEL.P, PID_VEL.I, PID_VEL.D, 100000, MOTOR_MAX_TORQUE),
      pid_steering_(PID_STEER.P, PID_STEER.I, PID_STEER.D, MOTOR_MAX_TORQUE / 2),
      tune_stb_(PID_STB.P, PID_STB.I, PID_STB.D),
      tune_vel_(PID_VEL.P, PID_VEL.I, PID_VEL.D),
      tune_steering_(PID_STEER.P, PID_STEER.I, PID_STEER.D),
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
static void init_motor(BLDCMotor *motor, BLDCDriver3PWM *driver, SensorType *sensor, InlineCurrentSense *cs=nullptr)
{
    // sensor参数设置
    sensor->init();
    motor->linkSensor(sensor);

    // driver参数设置
    driver->pwm_frequency = 20000;
    driver->voltage_power_supply = 7.4;
    driver->voltage_limit = 7.4;
    driver->init();
    motor->linkDriver(driver);

    if (cs == nullptr) {
        //FOC模型选择
        motor->foc_modulation = FOCModulationType::SpaceVectorPWM;
        motor->torque_controller = TorqueControlType::voltage;
        motor->controller = MotionControlType::torque;

        //速度低通滤波时间常数
        motor->LPF_velocity.Tf = 0.02f;
        motor->PID_velocity.output_ramp = 1000;
    } else {
        // current sense参数设置
        cs->init();
#if 1
        cs->gain_a *= -1;
        cs->gain_b *= -1;
        cs->skip_align = true;
#else
        cs->linkDriver(driver);
#endif
        motor->linkCurrentSense(cs);
        // FOC模型选择
        motor->torque_controller = TorqueControlType::foc_current;
        motor->controller = MotionControlType::torque;
        // PID参数设置
        motor->PID_current_q.P = 2;
        motor->PID_current_q.I = 0;
        motor->PID_current_d.P = 2;
        motor->PID_current_d.I = 0;
        motor->LPF_current_q.Tf = 0.002;
        motor->LPF_current_d.Tf = 0.002;
    }

    //最大电机限制电机
    motor->voltage_limit = 7.4;

    //初始化电机
    motor->init();
}

static int initFOC(BLDCMotor &motor, float offset=NOT_SET)
{
    int ret = 0;
    if (offset != NOT_SET) {
        ret = motor.initFOC(offset, Direction::CCW); // The direction is CCW
    } else {
        // recalibrate
        ret = motor.initFOC();
        if (ret == 1) {
            ESP_LOGI(TAG, "motor zero electric angle: %.2f", motor.zero_electric_angle);
        }
    }
    return ret;
}

void Motor::foc_timer_callback(void* arg) {
    Motor* self = static_cast<Motor*>(arg);
    // 电流环 4kHz
    self->motor_l.loopFOC();
    self->motor_l.move();
    self->motor_r.loopFOC();
    self->motor_r.move();
}

int Motor::start_foc_timer()
{
    // 创建 esp_timer，并传入 this 指针
    const esp_timer_create_args_t timer_args = {
        .callback = &Motor::foc_timer_callback,  // 静态回调
        .arg = this,
        .name = "foc_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &foc_timer_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(foc_timer_, FOC_TIMER_PERIOD_US));

    return 0;
}

int Motor::runBalanceTask()
{
    static size_t count = 0;
    static unsigned long idle_start_ms = 0;
    /* ---------- 外环状态 ---------- */
    static float target_pitch = mid_value_;
    static float steering_out = 0.0f;

    float pitch      = imu_->getPitch();
    float gyro_pitch = imu_->lowPassGyroPitch();
    float gyro_yaw   = imu_->lowPassGyroZ();

    if (count % 2 == 0) {
        /* 动态设置pid */
        bool is_remote_active = (fabs(throttle_) > 0.01f || fabs(steering_) > 0.01f);
        if (is_remote_active) {
            // 遥控运动态
            // 忽略速度环I
            pid_vel_.I = 0;
            pid_vel_.resetIntegral();
            // 直立环P降低一半，运动更丝滑
            pid_stb_.P = tune_stb_.P * 0.6f;

            // 刷新时间
            idle_start_ms = millis();
        } else {
            // 静止态
            if (millis() > idle_start_ms + BALANCE_ENABLE_STEERING_I_TIME) {
                // 恢复速度环I，原地锁定
                pid_vel_.I = tune_vel_.I;
                // 恢复直立环P，直立刚性
                pid_stb_.P = tune_stb_.P;
            }
        }

        /* ---------- 速度外环 ---------- */
        float speed_out = pid_vel_(lpf_throttle(throttle_) - (motor_l.shaft_velocity - motor_r.shaft_velocity));
        // speed_out = constrain(speed_out, -MAX_TILT_RAD, MAX_TILT_RAD);
        target_pitch = mid_value_ + speed_out;

        /* ---------- 转向 ---------- */
        steering_out = pid_steering_(
            lpf_steering(steering_),
            0.0f,
            gyro_yaw
        );
    }

    /* ---------- 姿态内环 ---------- */
    float balance_out = pid_stb_(target_pitch, pitch, gyro_pitch);

    motor_l.target = -(balance_out + steering_out);
    motor_r.target =  (balance_out - steering_out);

    count++;
    return 0;
}

void Motor::task()
{
    BotState last_state = bot_state_;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFreq = pdMS_TO_TICKS(2); // 500 Hz

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFreq);
        // update IMU
        imu_->update();
        checkBalanceStatus(imu_->getPitch());

        /* ---------- 状态切换（entry action，只执行一次） ---------- */
        if (last_state != bot_state_) {
            ESP_LOGI(TAG, "State changed: %d -> %d", last_state, bot_state_);

            switch (bot_state_) {
            case BOT_FALL:
                // [倒地状态]
                motor_l.target = 0;
                motor_r.target = 0;

                // 清除积分项，防止扶起来瞬间猛冲
                resetAllPid();
                motor_l.PID_current_q.reset();
                motor_l.PID_current_d.reset();
                motor_r.PID_current_q.reset();
                motor_r.PID_current_d.reset();
                break;

            case BOT_BALANCE:
                // [进入平衡状态]
                // 强烈建议在这里清一次 PID
                resetAllPid();
                break;

            case BOT_READY:
            case BOT_WAIT_BALANCE:
            case BOT_UNINIT:
            default:
                break;
            }

            last_state = bot_state_;
        }

        /* ---------- 状态运行逻辑（loop action，每个周期执行） ---------- */
        switch (bot_state_) {
        case BOT_BALANCE:
            runBalanceTask();
            break;

        case BOT_FALL:
        case BOT_WAIT_BALANCE:
        case BOT_READY:
        case BOT_UNINIT:
        default:
            // 空闲 / 安全态
            break;
        }
    }
}

static int vTaskSystemSync()
{
#if ( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
    TaskStatus_t * pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
    if( pxTaskStatusArray != NULL ) {
        configRUN_TIME_COUNTER_TYPE ulTotalTime;
        // 关键：触发 FreeRTOS 内部任务状态同步
        uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalTime );
        vPortFree( pxTaskStatusArray );
    }
#endif
    return 0;
}

int Motor::init()
{
    if (bot_state_ != BOT_UNINIT) {
        ESP_LOGW(TAG, "Init ignored: Motor is already inited (current: %d)", bot_state_);
        return -1;
    }
    ESP_LOGI(TAG, "Motor starting...");

    init_motor(&motor_l, &driver_l, &sensor_l, &cs_l);
    init_motor(&motor_r, &driver_r, &sensor_r, &cs_r);
    vTaskDelay(100);

    // get zero_electric_angle
    float l_offset = settings.GetFloat("l_offset", NOT_SET);
    float r_offset = settings.GetFloat("r_offset", NOT_SET);
    // get current sense offset
    float l_ia_offset = settings.GetFloat("l_ia_offset", NOT_SET);
    float l_ib_offset = settings.GetFloat("l_ib_offset", NOT_SET);
    float r_ia_offset = settings.GetFloat("r_ia_offset", NOT_SET);
    float r_ib_offset = settings.GetFloat("r_ib_offset", NOT_SET);
    // 左电机处理
    if (l_offset != NOT_SET && l_ia_offset != NOT_SET && l_ib_offset != NOT_SET) {
        ESP_LOGI(TAG, "Left motor zero_electric_angle = %f", l_offset);
        ESP_LOGI(TAG, "Left motor current sense offset: ia = %f, ib = %f", l_ia_offset, l_ib_offset);
        cs_l.offset_ia = l_ia_offset;
        cs_l.offset_ib = l_ib_offset;
        cs_l.need_calibration = false;
        initFOC(motor_l, l_offset);
    } else {
        ESP_LOGI(TAG, "Left motor offsets not set, doing auto calibration.");
        if (initFOC(motor_l) == 1) {
            settings.SetFloat("l_offset", motor_l.zero_electric_angle);
            settings.SetFloat("l_ia_offset", cs_l.offset_ia);
            settings.SetFloat("l_ib_offset", cs_l.offset_ib);
            ESP_LOGI(TAG, "Saved left motor offsets: zero_electric_angle = %f, ia = %f, ib = %f",
                    motor_l.zero_electric_angle, cs_l.offset_ia, cs_l.offset_ib);
        } else {
            ESP_LOGE(TAG, "Left motor auto calibration failed.");
            return -1;
        }
    }

    // 右电机处理
    if (r_offset != NOT_SET && r_ia_offset != NOT_SET && r_ib_offset != NOT_SET) {
        ESP_LOGI(TAG, "Right motor zero_electric_angle = %f", r_offset);
        ESP_LOGI(TAG, "Right motor current sense offset: ia = %f, ib = %f", r_ia_offset, r_ib_offset);
        cs_r.offset_ia = r_ia_offset;
        cs_r.offset_ib = r_ib_offset;
        cs_r.need_calibration = false;
        initFOC(motor_r, r_offset);
    } else {
        ESP_LOGI(TAG, "Right motor offsets not set, doing auto calibration.");
        if (initFOC(motor_r) == 1) {
            settings.SetFloat("r_offset", motor_r.zero_electric_angle);
            settings.SetFloat("r_ia_offset", cs_r.offset_ia);
            settings.SetFloat("r_ib_offset", cs_r.offset_ib);
            ESP_LOGI(TAG, "Saved right motor offsets: zero_electric_angle = %f, ia = %f, ib = %f",
                    motor_r.zero_electric_angle, cs_r.offset_ia, cs_r.offset_ib);
        } else {
            ESP_LOGE(TAG, "Right motor auto calibration failed.");
            return -1;
        }
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
            1);
        if (motor_task_handle_ == nullptr) {
            ESP_LOGE(TAG, "start motor_run task failed.");
            return -1;
        }
    }

#ifdef CONFIG_ENABLE_CONSOLE
    register_pid_cmd();
#endif

#ifdef CONFIG_PID_TUNER_ENABLE
    pid_tuner_start();
#endif

    bot_state_ = BOT_READY;
    ESP_LOGI(TAG, "State: UNINIT -> READY");

    return 0;
}

int Motor::start()
{
    if (bot_state_ == BOT_READY) {
        vTaskSystemSync(); // 重要!!!，这个会同步task，没有这个会导致simplefoc运行不正常

        start_foc_timer();

        bot_state_ = BOT_FALL;
        ESP_LOGI(TAG, "State: READY -> FALL (Control Loop Enabled)");
        ESP_LOGI(TAG, "Motor start.");

        return 0;
    } else {
        ESP_LOGW(TAG, "Start ignored: Motor is not in READY state (current: %d)", bot_state_);
        return -1;
    }
}

void Motor::stop()
{
    motor_l.target = 0;
    motor_r.target = 0;

    if (foc_timer_) {
        esp_timer_stop(foc_timer_);
        esp_timer_delete(foc_timer_);
        foc_timer_ = nullptr;
    }

    resetAllPid();
    motor_l.PID_current_q.reset();
    motor_l.PID_current_d.reset();
    motor_r.PID_current_q.reset();
    motor_r.PID_current_d.reset();

    driver_l.disable();
    driver_r.disable();

    bot_state_ = BOT_READY;

    ESP_LOGI(TAG, "Motor stop.");
}

void Motor::checkBalanceStatus(float pitch)
{
    static unsigned long wait_start_ts = 0;
    switch (bot_state_) {
    case BOT_FALL:
        // 进入可平衡区，pitch 在阈值范围内
        if (fabs(pitch) < BALANCE_PITCH_THRESHOLD) {
            bot_state_ = BOT_WAIT_BALANCE;
            wait_start_ts = millis();
            ESP_LOGI(TAG, "State: FALL -> WAIT_BALANCE");
        }
        break;

    case BOT_WAIT_BALANCE:
        if (fabs(pitch) >= BALANCE_PITCH_THRESHOLD) {
            bot_state_ = BOT_FALL; // 又掉出范围
            ESP_LOGI(TAG, "pitch: %f, State: WAIT_BALANCE -> FALL", pitch);
        } else if (millis() - wait_start_ts >= BALANCE_WAITTING_TIME) {
            bot_state_ = BOT_BALANCE; // 等待时间到了，进入平衡
            resetAllPid();
            ESP_LOGI(TAG, "State: WAIT_BALANCE -> BALANCE");
        }
        break;

    case BOT_BALANCE:
        if (fabs(pitch) >= BALANCE_PITCH_THRESHOLD) {
            bot_state_ = BOT_FALL; // 倒了，重新开始
            ESP_LOGI(TAG, "pitch: %f, State: BALANCE -> FALL", pitch);
        }
        break;
    default:
        break;
    }
}

void Motor::resetAllPid()
{
    pid_stb_.reset();
    pid_vel_.reset();
    pid_steering_.reset();
}

IPID* Motor::getPID(PIDType type)
{
    switch (type) {
    case PIDType::STB:
        return &pid_stb_;
    case PIDType::VEL:
        return &pid_vel_;
    case PIDType::STEER:
        return &pid_steering_;
    }
    return nullptr;
}

PIDParams* Motor::getPIDParam(PIDType type) {
    struct PIDParams *target = nullptr;
    switch (type) {
        case PIDType::STB:   target = &tune_stb_; break;
        case PIDType::VEL:   target = &tune_vel_; break;
        case PIDType::STEER: target = &tune_steering_; break;
    }

    return target;
}

void Motor::updatePIDParam(PIDType type, float p, float i, float d) {
    switch (type) {
        case PIDType::STB:
            pid_stb_.P = tune_stb_.P = p;
            pid_stb_.I = tune_stb_.I = i;
            pid_stb_.D = tune_stb_.D = d;
            break;

        case PIDType::VEL:
            pid_vel_.P = tune_vel_.P = p;
            pid_vel_.I = tune_vel_.I = i;
            pid_vel_.D = tune_vel_.D = d;
            break;

        case PIDType::STEER:
            pid_steering_.P = tune_steering_.P = p;
            pid_steering_.I = tune_steering_.I = i;
            pid_steering_.D = tune_steering_.D = d;
            break;
    }
    ESP_LOGI(TAG, "Tuned parameters synced for type %d", (int)type);
}

/* controlled by MCP */
void Motor::setMotion(float speed, float steering)
{
    float max_speed = SPEED_GEAR_MAX[static_cast<int>(speed_gear_)];
    speed = std::clamp(speed, -static_cast<float>(max_speed), static_cast<float>(max_speed));
    steering = std::clamp(steering, -static_cast<float>(MOTOR_MAX_STEERING), static_cast<float>(MOTOR_MAX_STEERING));

    throttle_ = speed;
    steering_ = steering;
    ESP_LOGI(TAG, "throttle: %.2f, steering %.2f.", throttle_, steering_);
}

void Motor::setSpeedGear(SpeedGear gear) {
    speed_gear_ = gear;
    auto& app = Application::GetInstance();
    switch (speed_gear_) {
        case SpeedGear::SLOW:
            ESP_LOGI(TAG, "Speed gear set to SLOW.");
            app.Alert(Lang::Strings::SPEED_GEAR, Lang::Strings::GEAR_LOW, "confident", Lang::Sounds::P3_GEAR_LOW);
            break;
        case SpeedGear::MEDIUM:
            ESP_LOGI(TAG, "Speed gear set to MEDIUM.");
            app.Alert(Lang::Strings::SPEED_GEAR, Lang::Strings::GEAR_MID, "confident", Lang::Sounds::P3_GEAR_MID);
            break;
        case SpeedGear::FAST:
            ESP_LOGI(TAG, "Speed gear set to FAST.");
            app.Alert(Lang::Strings::SPEED_GEAR, Lang::Strings::GEAR_HIGH, "confident", Lang::Sounds::P3_GEAR_HIGH);
            break;
        default:
            ESP_LOGW(TAG, "Unknown speed gear: %d", static_cast<int>(speed_gear_));
            break;
    }
}
float Motor::getMaxSpeedByGear() const {
    return SPEED_GEAR_MAX[static_cast<int>(speed_gear_)];
}

/* controlled by AI */
void Motor::move(direction_t dir, int distance_cm) {
    float distance_m = static_cast<float>(distance_cm) / 100.0f;
    float move_time_sec = distance_m / TARGET_SPEED_MPS;  // t = s / v

    // 开始行驶
    if (dir == FORWARD) {
        setMotion(-TARGET_SPEED_RADS, 0);
    } else if (dir == BACKWARD) {
        setMotion(TARGET_SPEED_RADS, 0);
    } else {
        ESP_LOGE(TAG, "Invalid direction for motor move.");
        return;
    }

    // 延时一段时间，表示运行这么久就可以了
    vTaskDelay(pdMS_TO_TICKS(move_time_sec * 1000));

    // 停车
    setMotion(0.0f, 0.0f);
}

void Motor::rotate(int angle) {
    constexpr int ROTATE_LOOP_DELAY_MS = 10;
    float target_yaw = imu_->getYaw() + angle;
    int elapsed = 0;

    while ((elapsed+=ROTATE_LOOP_DELAY_MS) < 5000) { // 5秒超时
        if (angle < 0) {
            setMotion(0, 30); // CW
        } else {
            setMotion(0, -30); // CCW
        }
        vTaskDelay(pdMS_TO_TICKS(ROTATE_LOOP_DELAY_MS));

        if (fabs(imu_->getYaw() - target_yaw) < 2.0f) {
            break;
        }
    }
    setMotion(0, 0);
}

void Motor::turnAround(void) {
    rotate(360);
}