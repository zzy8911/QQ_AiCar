#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "hal.h"
#include "esp_log.h"
#include "boards/self-balancing-robot/config.h"
#include "driver/spi_master.h"
#include "settings.h"
#include <esp_simplefoc.h>
#include "gyro_pid.h"

// #define pi 3.1415926
#define init_smooth 1000 // 该值越大，初始化越慢。以防受到干扰。
#define volt_limit 5.0000

extern GyroPID pid_stb;
extern PIDController pid_vel;
extern float g_mid_value;
extern GyroPID pid_steering;

#endif // 