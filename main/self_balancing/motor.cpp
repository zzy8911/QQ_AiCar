#include "motor.h"
#include <memory>

#define TAG "Motor"

bool g_system_calibration = false;

BLDCMotor motor_0 = BLDCMotor(7);
BLDCDriver3PWM driver_0 = BLDCDriver3PWM(MO0_1, MO0_2, MO0_3); // 12, 11, 10，左电机
BLDCMotor motor_1 = BLDCMotor(7);
BLDCDriver3PWM driver_1 = BLDCDriver3PWM(MO1_1, MO1_2, MO1_3); // 40, 39, 38，右电机

enum BALANCE_STATUS {
    BALANCE_OFF,
    BALANCE_WATTING,
    BALANCE_RUNNING,
};

static BALANCE_STATUS g_balance_status = BALANCE_OFF;
// low pass filters for user commands - throttle(油门) and steering
LowPassFilter lpf_throttle(0.5);
LowPassFilter lpf_steering(0.5);

#define MOTOR_MAX_TORQUE 45.0f
#define BALANCE_WAITTING_TIME  1000
#define BALANCE_ENABLE_STEERING_I_TIME  3000
#define BALANCE_STOP_PITCH_OFFSET 40
// control algorithm parametersw
// stabilisation pid
// 初始值 P0.3 D: 0.02  -- 0.18 0.024
// PIDController pid_stb(0.03, 0, 0.002, 100000, MOTOR_MAX_TORQUE); // *0.6
GyroPID pid_stb(0.013, 0, -0.0009, MOTOR_MAX_TORQUE); // -0.04, 0, -0.002, *0.6
#define PID_VEL_P (1.22)
#define PID_VEL_I (0.006)
#define PID_VEL_D (0)
PIDController pid_vel(PID_VEL_P, PID_VEL_I, PID_VEL_D, 100000, MOTOR_MAX_TORQUE);
PIDController pid_vel_tmp(PID_VEL_P, PID_VEL_I, PID_VEL_D, 100000, MOTOR_MAX_TORQUE);
GyroPID pid_steering(0.01, 0, 0.00, MOTOR_MAX_TORQUE / 2);

float g_mid_value = -1.0f; // 偏置参数
float g_throttle = 0;
float g_steering = 0;

static StackType_t* motor_task_stack_ = nullptr;
static StaticTask_t motor_task_tcb_;

i2c_master_dev_handle_t i2c_device_0 = nullptr; // 左电机编码器
i2c_master_dev_handle_t i2c_device_1 = nullptr; // 右电机编码器
#define AS5600_I2C_ADDR 0x36
static float read_data_callback(void) {
    uint8_t raw_angle_buf[2] = {0};
    uint8_t reg = 0x0C; // AS5600 angle register

    if (xSemaphoreTake(HAL::i2c_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        esp_err_t ret = i2c_master_transmit_receive(i2c_device_0, &reg, 1, raw_angle_buf, 2, 100);
        xSemaphoreGive(HAL::i2c_mutex);
        if (ESP_OK == ret) {
            uint16_t raw_angle = (uint16_t)(raw_angle_buf[0] << 8 | raw_angle_buf[1]);
            float angle = (((int)raw_angle & 0b0000111111111111) / 4096.0f) * (2 * PI);

            // ESP_LOGI(TAG, "AS5600 sensor 0 angle: %f", angle);
            return angle;
        } else {
            ESP_LOGE(TAG, "Failed to read AS5600 sensor 0 angle data");
            return 0.0f;
        }
    } else {
        ESP_LOGE(TAG, "Failed to take I2C mutex");
        return 0.0f;
    }
}
static void init_callback(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(HAL::get_i2c_bus(ENCODER_I2C_BUS), &dev_cfg, &i2c_device_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "AS5600 I2C sensor 0 initialized successfully");
}
std::unique_ptr<GenericSensor> sensor_0 = nullptr;

static float read_data_callback_1(void) {
    uint8_t raw_angle_buf[2] = {0};
    uint8_t reg = 0x0C; // AS5600 angle register

    if (xSemaphoreTake(HAL::i2c_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        esp_err_t ret = i2c_master_transmit_receive(i2c_device_1, &reg, 1, raw_angle_buf, 2, 100);
        xSemaphoreGive(HAL::i2c_mutex);
        if (ESP_OK == ret) {
            uint16_t raw_angle = (uint16_t)(raw_angle_buf[0] << 8 | raw_angle_buf[1]);
            float angle = (((int)raw_angle & 0b0000111111111111) / 4096.0f) * (2 * PI);

            // ESP_LOGI(TAG, "AS5600 sensor 1 angle: %f", angle);
            return angle;
        } else {
            ESP_LOGE(TAG, "Failed to read AS5600 sensor 1 angle data");
            return 0.0f;
        }
    } else {
        ESP_LOGE(TAG, "Failed to take I2C mutex");
        return 0.0f;
    }
}
static void init_callback_1(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(HAL::get_i2c_bus(IMU_ENCODER_I2C_BUS), &dev_cfg, &i2c_device_1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "AS5600 I2C sensor 1 initialized successfully");
}
std::unique_ptr<GenericSensor> sensor_1 = nullptr;

static void motor_reset_all_pid()
{
    pid_stb.reset();
    pid_vel.reset();
    pid_steering.reset();
}

static int check_balance_status(float mpu_pitch)
{
    static unsigned long start_wait = 0;
    
    if (abs(mpu_pitch - g_mid_value) > BALANCE_STOP_PITCH_OFFSET) {
        g_balance_status = BALANCE_OFF;
        return -1;
    }

    if (g_balance_status == BALANCE_RUNNING) {
        return 0;
    }

    if (g_balance_status == BALANCE_OFF) {
        g_balance_status = BALANCE_WATTING;
        start_wait = millis();
        return -1;
    }

    if (g_balance_status == BALANCE_WATTING) {
        if (millis() < start_wait + BALANCE_WAITTING_TIME) {
            return -1;
        }
    }
    motor_reset_all_pid();
    g_balance_status = BALANCE_RUNNING;
    return 0;
}

static int run_balance_task(BLDCMotor *motor_l, BLDCMotor *motor_r,
                                float throttle, float steering)
{
    // float voltage_control;
    static unsigned long ctlr_start_ms = 0; 
    int rc = 0;
    float speed = 0;
    float speed_adj  = 0;
    float stb_adj = 0;
    float steering_adj = 0;
    float all_adj = 0;

    float mpu_pitch = HAL::imu_get_pitch();
    // ESP_LOGI(TAG, "mpu_pitch: %.2f, throttle: %.2f, steering: %.2f", mpu_pitch, throttle, steering);
    // float mpu_yaw = HAL::imu_get_yaw();
    // float gyro_z = HAL::imu_get_gyro_z();
    rc = check_balance_status(mpu_pitch);
    if (rc) {
        motor_l->target = 0;
        motor_r->target = 0;
        goto out;
    }

    // When rotating in the same direction, one has a positive sign and the other negative, so the speeds are subtracted.
    speed = (motor_l->shaft_velocity - motor_r->shaft_velocity) / 2.0f;
#if 1
    /* Cascade PID */
    speed_adj = pid_vel(lpf_throttle(throttle) - speed); // speed_adj is the target angle (in degrees).
    // ESP_LOGI(TAG, "speed: %.2f, speed_adj: %.2f", speed, speed_adj);
    speed_adj += g_mid_value; // add mid value offset
    all_adj = pid_stb(speed_adj, mpu_pitch, HAL::lowPassGyroX());
#else
    /* Parallel PID */
    /*
     * 角速度环:
     * MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
     * wPIDOutput = PID_W(&wPID, gyroy);
     * Set_Pwm(wPIDOutput, wPIDOutput);
     */
    // stb_adj = pid_stb(g_mid_value - mpu_pitch);
    stb_adj = pid_stb(g_mid_value, mpu_pitch, HAL::lowPassGyroX());
    if (throttle != 0) {
        pid_vel.I = 0;
        ctlr_start_ms = millis();
    } else {
        if (millis() > ctlr_start_ms + BALANCE_ENABLE_STEERING_I_TIME) {
            pid_vel.I = pid_vel_tmp.I;
        }
    }

    speed_adj = pid_vel(lpf_throttle(throttle) - speed);
    all_adj = stb_adj + speed_adj;
#endif

    // steering
    steering_adj = pid_steering(lpf_steering(steering), 0.0f, HAL::lowPassGyroZ());

    motor_l->target = -(all_adj + steering_adj);
    motor_r->target = (all_adj - steering_adj);
out:
    motor_l->move();
    motor_r->move();
    return rc;
}

static int motor_task_mode_update(int &mode, bool &is_changed)
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

TaskHandle_t handleTaskMotor;
void motor_task(void *pvParameters)
{
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int motor_task = BOT_RUNNING_MODE;
    bool is_task_changed = false;

    // Execute every 6 milliseconds
    while(1) {
        is_task_changed = false;

        HAL::imu_update(); // 更新IMU数据

        // sensor_0->update();
        motor_0.loopFOC();
        // Serial.printf("angle: %f\n", motor_1.shaft_angle);
        // sensor_1->update();
        motor_1.loopFOC();

        motor_task_mode_update(motor_task, is_task_changed);
        switch(motor_task) {
        case BOT_RUNNING_MODE:
            motor_0.move(0);
            motor_1.move(0);
            break;
        case BOT_RUNNING_BALANCE:
            run_balance_task(&motor_0, &motor_1, g_throttle, g_steering);
            break;
        default:
            break;
        }

        // motor_0.monitor();
        // motor_1.monitor();
        // Serial.println(motor_config[id].position);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void init_motor(BLDCMotor *motor, BLDCDriver3PWM *driver, GenericSensor *sensor)
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
    // motor->modulation_centered = 1.0;
    //运动控制模式设置
    motor->torque_controller = TorqueControlType::voltage;
    motor->controller = MotionControlType::torque;

    // 速度PI环设置
    motor->PID_velocity.P = 0;
    motor->PID_velocity.I = 0;
    motor->PID_velocity.D = 0;
    //速度低通滤波时间常数
    motor->LPF_velocity.Tf = 0.02f;
    motor->PID_velocity.output_ramp = 1000;
    // motor->PID_velocity.limit = MOTOR_MAX_SPEED; // rad/s
    //最大电机限制电机
    motor->voltage_limit = 8;

    //设置最大速度限制
    // motor->velocity_limit = MOTOR_MAX_SPEED;

    // motor->monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;

#ifdef XK_WIRELESS_PARAMETER
    motor->useMonitoring(HAL::get_wl_tuning());
#else
    Serial.begin(115200);
    motor->useMonitoring(Serial);
#endif
    //初始化电机
    motor->init();
    // motor->initFOC();
    // motor->monitor_downsample = 100;  // disable monitor at first - optional
}

void motor_initFOC(BLDCMotor *motor, float offset)
{
    if (offset > 0) {
        ESP_LOGI(TAG, "has a offset value %.2f.", offset);
        Direction foc_direction = Direction::CW;
        motor->initFOC(offset, foc_direction);
    } else {
        if (motor->initFOC()) {
            ESP_LOGI(TAG, "motor zero electric angle: %.2f", motor->zero_electric_angle);
        }
    }
}

void HAL::motor_init(void)
{
    bool has_set_offset = false;
    Settings settings("motor", true);
    ESP_LOGI(TAG, "Motor starting...");

    sensor_0 = std::make_unique<GenericSensor>(read_data_callback, init_callback);
    sensor_1 = std::make_unique<GenericSensor>(read_data_callback_1, init_callback_1);
    init_motor(&motor_0, &driver_0, sensor_0.get());
    init_motor(&motor_1, &driver_1, sensor_1.get());
    vTaskDelay(100);

    ESP_LOGI(TAG, "[motor]: calibration %s", g_system_calibration?"true":"false");
    if (g_system_calibration == false) {
        float l_offset = settings.GetFloat("l_offset", 0);
        float r_offset = settings.GetFloat("r_offset", 0);
        if (l_offset != 0 || r_offset != 0) {
            ESP_LOGI(TAG, "[motor]: set offset %f, %f", l_offset, r_offset);
            motor_initFOC(&motor_0, l_offset);
            motor_initFOC(&motor_1, r_offset);
            has_set_offset = true;
        } else {
            ESP_LOGI(TAG, "motor: get config failed, try auto calibration.");
        }
    }
    if (!has_set_offset) {
        motor_initFOC(&motor_0, 0);
        motor_initFOC(&motor_1, 0);

        settings.SetFloat("l_offset", motor_0.zero_electric_angle); // 0.9157872
        settings.SetFloat("r_offset", motor_1.zero_electric_angle); // 0.6583844
    }
    
    ESP_LOGI(TAG, "Motor ready.");
    // ESP_LOGI(TAG, "Set the target velocity using serial terminal:");

    motor_task_stack_ = (StackType_t*) heap_caps_malloc(4096 * sizeof(StackType_t), MALLOC_CAP_SPIRAM);
    handleTaskMotor = xTaskCreateStaticPinnedToCore(
        motor_task,
        "MotorThread",
        4096,
        nullptr,
        10,
        motor_task_stack_,
        &motor_task_tcb_,
        1); // Motor: CORE 1
    if (handleTaskMotor == nullptr) {
        ESP_LOGE(TAG, "start motor_run task failed.");
        // return -1;
    }
}

void HAL::motor_set_speed(float speed, float steering)
{
    if (g_throttle != speed || g_steering != steering) {
        // ESP_LOGE(TAG, "speed: %d steering %d.", speed, steering);

        if (speed < -MOTOR_MAX_SPEED) {
            speed = -MOTOR_MAX_SPEED;
        }
        if (speed > MOTOR_MAX_SPEED) {
            speed = MOTOR_MAX_SPEED;
        }

        if (steering < -BOT_MAX_STEERING) {
            steering = -BOT_MAX_STEERING;
        }
        if (steering > BOT_MAX_STEERING) {
            steering = BOT_MAX_STEERING;
        }

        g_throttle = (float)speed;
        g_steering = (float)-steering;
        // ESP_LOGE(TAG, "throttle: %.2f steering %.2f.", g_throttle, g_steering);
    }
}