#include "motor.h"
#include <memory>
#include <algorithm>

#define TAG "Motor"

Motor::Motor()
    : motor_l(7), motor_r(7),
      driver_l(MO0_1, MO0_2, MO0_3), driver_r(MO1_1, MO1_2, MO1_3),
      sensor_l(SPI2_HOST, ENCODER_SCK, ENCODER_MISO, GPIO_NUM_NC, ENCODER_CS0),
      sensor_r(SPI2_HOST, ENCODER_SCK, ENCODER_MISO, GPIO_NUM_NC, ENCODER_CS1),
      cs_l(0.005, 50.0, 4, 5, NOT_SET),
      cs_r(0.005, 50.0, 6, 7, NOT_SET),
      settings("motor", true),
      pid_stb_(PID_STB.P, PID_STB.I, PID_STB.D, MOTOR_MAX_TORQUE),
      pid_vel_(PID_VEL.P, PID_VEL.I, PID_VEL.D, 100000, MOTOR_MAX_TORQUE),
      pid_vel_tmp_(PID_VEL.P, PID_VEL.I, PID_VEL.D, 100000, MOTOR_MAX_TORQUE),
      pid_steering_(0.02, 0, 0.001, MOTOR_MAX_TORQUE / 2),
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
        cs->gain_a *= -1;
        cs->gain_b *= -1;
        cs->skip_align = true;
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

static void initFOC(BLDCMotor &motor, float offset)
{
    if (offset > 0) {
        ESP_LOGI(TAG, "has a offset value %.2f.", offset);
        Direction foc_direction = Direction::CW;
        motor.initFOC();
    } else {
        if (motor.initFOC()) {
            ESP_LOGI(TAG, "motor zero electric angle: %.2f", motor.zero_electric_angle);
        }
    }
}

void Motor::foc_timer_callback(void* arg) {
    Motor* self = static_cast<Motor*>(arg);
    xSemaphoreGive(self->foc_sem_);
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

int Motor::init()
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

    foc_sem_ = xSemaphoreCreateBinary();
    if (foc_sem_ == nullptr) {
        ESP_LOGE(TAG, "FOC semaphore create failed.");
        return -1;
    }

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
            20,
            motor_task_stack_,
            &motor_task_tcb_,
            1); // Motor: CORE 1
        if (motor_task_handle_ == nullptr) {
            ESP_LOGE(TAG, "start motor_run task failed.");
            return -1;
        }
    }

    start_foc_timer();

    return 0;
}

void Motor::checkBalanceStatus(float pitch, BOT_STATUS &bot_state)
{
    static unsigned long wait_start_ts = 0;
    switch (bot_state) {
    case BOT_FALL:
        // 进入可平衡区，pitch 在阈值范围内
        if (fabs(pitch) < BALANCE_PITCH_THRESHOLD) {
            bot_state = BOT_WAIT_BALANCE;
            wait_start_ts = millis();
        }
        break;

    case BOT_WAIT_BALANCE:
        if (fabs(pitch) >= BALANCE_PITCH_THRESHOLD) {
            bot_state = BOT_FALL; // 又掉出范围
            ESP_LOGI(TAG, "pitch: %f, BOT_FALL", pitch);
        } else if (millis() - wait_start_ts >= BALANCE_WAITTING_TIME) {
            bot_state = BOT_BALANCE; // 等待时间到了，进入平衡
            resetAllPid();
            ESP_LOGI(TAG, "BOT_BALANCE");
        }
        break;

    case BOT_BALANCE:
        if (fabs(pitch) >= BALANCE_PITCH_THRESHOLD) {
            bot_state = BOT_FALL; // 倒了，重新开始
            ESP_LOGI(TAG, "pitch: %f, BOT_FALL", pitch);
        }
        break;
    default:
        bot_state = BOT_FALL;
        break;
    }
}

void Motor::resetAllPid()
{
    pid_stb_.reset();
    pid_vel_.reset();
    pid_steering_.reset();
}

int Motor::runBalanceTask()
{
    // float voltage_control;
    static unsigned long ctlr_start_ms = 0; 
    int rc = 0;
    float speed = 0;
    static size_t count = 0;

    float mpu_pitch = imu_->getPitch();

    /* Parallel PID */
    stb_adj_ = pid_stb_(mid_value_, mpu_pitch, imu_->lowPassGyroPitch());

    /* every 4th loop, run speed and steering PID */
    // if (count % 4 == 0) {
    //     // speed
    //     if (throttle_ != 0) {
    //         pid_vel_.I = 0;
    //         ctlr_start_ms = millis();
    //     } else {
    //         if ((unsigned long)(millis() - ctlr_start_ms) > BALANCE_ENABLE_STEERING_I_TIME) {
    //             pid_vel_.I = pid_vel_tmp_.I;
    //         }
    //     }
    //     // When rotating in the same direction, one has a positive sign and the other negative, so the speeds are subtracted.
    //     speed = (motor_l.shaft_velocity - motor_r.shaft_velocity) / 2.0f;
    //     speed_adj_ = pid_vel_(lpf_throttle(throttle_) - speed);

    //     // steering
    //     steering_adj_ = pid_steering_(lpf_steering(steering_), 0.0f, imu_->lowPassGyroZ());
    // }

    motor_l.target = -(stb_adj_ + speed_adj_ + steering_adj_);
    motor_r.target = (stb_adj_ + speed_adj_ - steering_adj_);

    count++;

    return rc;
}

void Motor::task()
{
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    BOT_STATUS bot_mode = BOT_FALL;

    while(1) {
        xSemaphoreTake(foc_sem_, portMAX_DELAY);

        // --- 高频循环 (FOC) ---
        motor_l.loopFOC();
        motor_l.move();
        motor_r.loopFOC();
        motor_r.move();

        imu_->update(); // 更新IMU数据
        checkBalanceStatus(imu_->getPitch(), bot_mode);
        if (bot_mode != BOT_BALANCE) {
            motor_l.target = 0;
            motor_r.target = 0;
        } else if (bot_mode == BOT_BALANCE) {
            runBalanceTask();
        }
    }
}

void Motor::setMotion(float speed, float steering)
{
    speed = std::clamp(speed, -static_cast<float>(MOTOR_MAX_SPEED), static_cast<float>(MOTOR_MAX_SPEED));
    steering = std::clamp(steering, -static_cast<float>(MOTOR_MAX_STEERING), static_cast<float>(MOTOR_MAX_STEERING));

    throttle_ = speed;
    steering_ = steering;
    ESP_LOGI(TAG, "throttle: %.2f, steering %.2f.", throttle_, steering_);
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