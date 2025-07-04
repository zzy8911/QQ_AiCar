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

XboxSeriesXControllerESP32_asukiaaa::Core* xboxController = nullptr;

#define TAG "Controller"

int g_bot_ctrl_type = BOT_CONTROL_TYPE_AI;

static ButtonEvent btn_a(5000);
static ButtonEvent btn_b(5000);
static ButtonEvent btn_dir_up(5000);
static ButtonEvent btn_dir_down(5000);
static ButtonEvent btn_dir_left(5000);
static ButtonEvent btn_dir_right(5000);


static inline bool btn_a_is_push(void)
{
    return (xboxController->xboxNotif.btnA == true);
}

static inline bool btn_b_is_push(void)
{
    return (xboxController->xboxNotif.btnB == true);
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
        g_bot_ctrl_type++;
        if (g_bot_ctrl_type >= BOT_CONTROL_TYPE_MAX) {
            g_bot_ctrl_type = BOT_CONTROL_TYPE_AI;
        }
        ESP_LOGI(TAG, "channge to %d control type", g_bot_ctrl_type);
    }
}

static void controller_btn_b_handler(ButtonEvent* btn, int event)
{
    // DBot &dbot = DBot::getInstance();
    // if (event == ButtonEvent::EVENT_PRESSED) {
    //     g_bot_ctrl_type = BOT_CONTROL_TYPE_AI;
    //     dbot.spin(90);
    // }
}

static void controller_btn_dir_up_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
        
    }
}

static void controller_btn_dir_down_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
    }
}

static void controller_btn_dir_right_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
    }
}

static void controller_btn_dir_left_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
    }
}


static long _map(long x, long in_min, long in_max, long out_min, long out_max) {
    const long run = in_max - in_min;
    if(run == 0){
        ESP_LOGE(TAG, "map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const long rise = out_max - out_min;
    const long delta = x - in_min;
    return (delta * rise) / run + out_min;
}


static void controller_set_motor_status(void)
{
    int speed = 0, steering = 0;
    static int last_speed = 0, last_steering = 0;

    if (g_bot_ctrl_type != BOT_CONTROL_TYPE_JOYSTICKS) {
        return;
    }
    
    // 左摇杆垂直控制速度，右摇杆水平控制方向
    speed = _map(xboxController->xboxNotif.joyLVert, 0, 65535, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    steering = _map(xboxController->xboxNotif.joyRHori, 0, 65535, -BOT_MAX_STEERING, BOT_MAX_STEERING);

    // if (speed == 0 && last_speed == 0 && steering == 0 && last_steering == 0) {
    //     // no change
    //     return;
    // }
    // last_speed = speed;
    // last_steering = steering;
    HAL::motor_set_speed(speed, steering);
}

void controller_update_task(void *parameter)
{
    while(1) {
        xboxController->onLoop();
        if (xboxController->isConnected()) {
            if (xboxController->isWaitingForFirstNotification()) {
                ESP_LOGI(TAG, "waiting for first notification");
            } else {
                // Serial.print(xbox_string());
                // demoVibration();
                // demoVibration_2();
                btn_a.EventMonitor(btn_a_is_push());
                btn_b.EventMonitor(btn_b_is_push());
                btn_dir_up.EventMonitor(btn_dir_up_is_push());
                btn_dir_down.EventMonitor(btn_dir_down_is_push());
                btn_dir_left.EventMonitor(btn_dir_left_is_push());
                btn_dir_right.EventMonitor(btn_dir_right_is_push());

                controller_set_motor_status();
            }
        } else {
            // Serial.println("not connected");
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
    btn_dir_up.EventAttach(controller_btn_dir_up_handler);
    btn_dir_down.EventAttach(controller_btn_dir_down_handler);
    btn_dir_left.EventAttach(controller_btn_dir_left_handler);
    btn_dir_right.EventAttach(controller_btn_dir_right_handler);
    xTaskCreatePinnedToCore(
        controller_update_task,
        "ControllerTask",
        4096,
        NULL,
        6,
        NULL,
        0); // Xbox: CORE 0
}
