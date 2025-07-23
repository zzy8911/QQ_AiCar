/**
  * @file    controller.h
  * @author  dingmos
  * @version V0.0.1
  * @date    2025-02-16
  * @brief   结合 button 库解析遥控器状态。
*/

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__
#define DEFAULTU_BLE_ADDR "97:ff:96:cc:36:7f"
void controller_init(const char *ble_addr);

#endif