/**
  * @file    controller.h
  * @author  dingmos
  * @version V0.0.1
  * @date    2025-02-16
  * @brief   结合 button 库解析遥控器状态。
*/

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__
#define DEFAULTU_BLE_ADDR "34:0a:db:0d:0f:37"
void controller_init(const char *ble_addr);

enum bot_control_type {
    BOT_CONTROL_TYPE_AI,
    BOT_CONTROL_TYPE_JOYSTICKS,
    BOT_CONTROL_TYPE_MAX,
};

#endif