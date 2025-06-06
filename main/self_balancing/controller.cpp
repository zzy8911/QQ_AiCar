/**
  * @file    controller.h
  * @author  dingmos
  * @version V0.0.1
  * @date    2025-02-16
  * @brief   结合 button 库解析遥控器状态。
*/

#include "hal/hal.h"
#include "ble_ctrl.h"
#include "button_event.h"
#include <Arduino.h>
#include "bot.h"

static BLECtrl ble_ctrl;
static BLEControllerNotificationParser *ble_parser;

static ButtonEvent btn_a(5000);
static ButtonEvent btn_b(5000);
static ButtonEvent btn_dir_up(5000);
static ButtonEvent btn_dir_down(5000);
static ButtonEvent btn_dir_left(5000);
static ButtonEvent btn_dir_right(5000);


static inline bool btn_a_is_push(void)
{
    ble_parser = ble_ctrl.get_status();
    return (ble_parser->btnA == 1);
}

static inline bool btn_b_is_push(void)
{
    ble_parser = ble_ctrl.get_status();
    return (ble_parser->btnB == 1);
}

static inline bool btn_dir_up_is_push(void)
{
    ble_parser = ble_ctrl.get_status();
    return (ble_parser->btnDirUp == 1);
}

static inline bool btn_dir_down_is_push(void)
{
    ble_parser = ble_ctrl.get_status();
    return (ble_parser->btnDirDown == 1);
}

static inline bool btn_dir_left_is_push(void)
{
    ble_parser = ble_ctrl.get_status();
    return (ble_parser->btnDirLeft == 1);
}

static inline bool btn_dir_right_is_push(void)
{
    ble_parser = ble_ctrl.get_status();
    return (ble_parser->btnDirRight == 1);
}

static void controller_btn_a_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED) {
        g_bot_ctrl_type++;
        if (g_bot_ctrl_type >= BOT_CONTROL_TYPE_MAX) {
            g_bot_ctrl_type = BOT_CONTROL_TYPE_AI;
        }
        log_i("channge to %d control type", g_bot_ctrl_type);
    }
}

static void controller_btn_b_handler(ButtonEvent* btn, int event)
{
    DBot &dbot = DBot::getInstance();
    if (event == ButtonEvent::EVENT_PRESSED) {
        g_bot_ctrl_type = BOT_CONTROL_TYPE_AI;
        dbot.spin(90);
    }
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
        log_e("map(): Invalid input range, min == max");
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
    
    ble_parser = ble_ctrl.get_status();

    speed = _map(ble_parser->joyLVert, 0, 256, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    steering = _map(ble_parser->joyRHori, 0, 256, -BOT_MAX_STEERING, BOT_MAX_STEERING);

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
        btn_a.EventMonitor(btn_a_is_push());
        btn_b.EventMonitor(btn_b_is_push());
        btn_dir_up.EventMonitor(btn_dir_up_is_push());
        btn_dir_down.EventMonitor(btn_dir_down_is_push());
        btn_dir_left.EventMonitor(btn_dir_left_is_push());
        btn_dir_right.EventMonitor(btn_dir_right_is_push());

        ble_ctrl.loop();

        controller_set_motor_status();

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    // __IntervalExecute(controller_set_motor_status(), 10);
}


void controller_init(const char *ble_addr)
{
    ble_ctrl.setup(ble_addr);

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
        0,
        NULL,
        0);
}



