/**
  * @file    controller.h
  * @author  dingmos
  * @version V0.0.1
  * @date    2025-02-16
  * @brief   结合 button 库解析遥控器状态。
*/

#include "hal.h"
#include "controller.h"
#include "XboxController/XboxSeriesXControllerESP32_asukiaaa.hpp"
#include "button/button_event.h"
#include "motor.h"

XboxSeriesXControllerESP32_asukiaaa::Core* xboxController = nullptr;

#define TAG "Controller"
// #define DEBUG

static ButtonEvent btn_a(5000);
static ButtonEvent btn_b(5000);
static ButtonEvent btn_x(5000);
static ButtonEvent btn_dir_up(5000);
static ButtonEvent btn_dir_down(5000);
static ButtonEvent btn_dir_left(5000);
static ButtonEvent btn_dir_right(5000);

static StackType_t* controller_update_task_stack_ = nullptr;
static StaticTask_t controller_update_task_tcb_;

static int gear = 0;

static inline bool btn_a_is_push(void)
{
    return (xboxController->xboxNotif.btnA == true);
}

static inline bool btn_b_is_push(void)
{
    return (xboxController->xboxNotif.btnB == true);
}

static inline bool btn_x_is_push(void)
{
    return (xboxController->xboxNotif.btnX == true);
}

static inline bool btn_dir_up_is_push(void)
{
    return (xboxController->xboxNotif.btnDirUp == true);
}

static inline bool btn_dir_down_is_push(void)
{
    return (xboxController->xboxNotif.btnDirDown == true);
}

static inline bool btn_dir_left_is_push(void)
{
    return (xboxController->xboxNotif.btnDirLeft == true);
}

static inline bool btn_dir_right_is_push(void)
{
    return (xboxController->xboxNotif.btnDirRight == true);
}

static void controller_btn_a_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
    }
}

static void controller_btn_b_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
    }
}

static void controller_btn_x_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
    }
}

static void controller_btn_dir_up_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
        gear++;
        if (gear >= SpeedGear::MAX_SPEED) {
            gear = SpeedGear::MAX_SPEED-1;
        }
        Motor::getInstance().setSpeedGear((SpeedGear)gear);
    }
}

static void controller_btn_dir_down_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
        gear--;
        if (gear < 0) {
            gear = SpeedGear::SLOW;
        }
        Motor::getInstance().setSpeedGear((SpeedGear)gear);
    }
}

static void controller_btn_dir_left_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
    }
}

static void controller_btn_dir_right_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
    }
}

static float _map(long x, long in_min, long in_max, long out_min, long out_max) {
    const long run = in_max - in_min;
    if(run == 0){
        ESP_LOGE(TAG, "map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const long rise = out_max - out_min;
    const long delta = x - in_min;

    float result = ((float)delta * (float)rise) / (float)run + (float)out_min;

    return result;
}

static void controller_set_motor_status(void)
{
    float speed = 0, steering = 0;
    static float last_speed = 0, last_steering = 0;
    static long last_update_time = 0; // 只要在控制，该值就会实时更新
    static bool already_stopped = false;
    long now = millis();

    // 左摇杆垂直控制速度，右摇杆水平控制方向
    float max_speed = Motor::getInstance().getMaxSpeedByGear();
    speed = _map(xboxController->xboxNotif.joyLVert, 0, 65535, -max_speed, max_speed);
    steering = _map(xboxController->xboxNotif.joyRHori, 0, 65535, -MOTOR_MAX_STEERING, MOTOR_MAX_STEERING);

    if (fabs(speed - last_speed) > 0.1f || fabs(steering - last_steering) > 0.1f) {
        last_speed = speed;
        last_steering = steering;
        last_update_time = now;
        already_stopped = false;
        Motor::getInstance().setMotion(speed, steering);
    } else {
        // 没有变化：判断是否持续超过3秒未更新
        if ((now - last_update_time) > 5000) {
            if (((fabs(speed)>0.01f)&&(fabs(speed)!=max_speed)) ||
                    ((fabs(steering)>0.01f)&&(fabs(steering)!=MOTOR_MAX_STEERING))) {
                // 非0状态下卡住，判定为异常断开
                if (!already_stopped) {
                    ESP_LOGW(TAG, "maybe disconnected (values not updated for 5s), stop motor");
                    Motor::getInstance().setMotion(0, 0);
                    already_stopped = true;
                }
            }
        }
    }
}

void controller_update_task(void *parameter)
{
    while(1) {
        xboxController->onLoop(); // this will effect softAP feature!!!
        if (xboxController->isConnected()) {
            if (xboxController->isWaitingForFirstNotification()) {
                ESP_LOGI(TAG, "waiting for first notification");
            } else {
                // Serial.print(xbox_string());
                // demoVibration();
                // demoVibration_2();
                btn_a.EventMonitor(btn_a_is_push());
                btn_b.EventMonitor(btn_b_is_push());
                btn_x.EventMonitor(btn_x_is_push());
                btn_dir_up.EventMonitor(btn_dir_up_is_push());
                btn_dir_down.EventMonitor(btn_dir_down_is_push());
                btn_dir_left.EventMonitor(btn_dir_left_is_push());
                btn_dir_right.EventMonitor(btn_dir_right_is_push());

                controller_set_motor_status();
            }
        } else {
            // ESP_LOGI(TAG, "not connected");
            // if (xboxController->getCountFailedConnection() > 2)
            // {
            //   ESP.restart();
            // }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    // __IntervalExecute(controller_set_motor_status(), 10);
}


void controller_init(const char *ble_addr)
{
    xboxController = new XboxSeriesXControllerESP32_asukiaaa::Core(ble_addr);
    xboxController->begin();

    btn_a.EventAttach(controller_btn_a_handler);
    btn_b.EventAttach(controller_btn_b_handler);
    btn_x.EventAttach(controller_btn_x_handler);
    btn_dir_up.EventAttach(controller_btn_dir_up_handler);
    btn_dir_down.EventAttach(controller_btn_dir_down_handler);
    btn_dir_left.EventAttach(controller_btn_dir_left_handler);
    btn_dir_right.EventAttach(controller_btn_dir_right_handler);

    controller_update_task_stack_ = (StackType_t*) heap_caps_malloc(4096 * sizeof(StackType_t), MALLOC_CAP_SPIRAM);
    xTaskCreateStaticPinnedToCore(
        controller_update_task,
        "ControllerTask",
        4096,
        NULL,
        6,
        controller_update_task_stack_,
        &controller_update_task_tcb_,
        0); // Xbox: CORE 0
}
