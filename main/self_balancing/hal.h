#ifndef __HAL_H__
#define __HAL_H__
// #include "button_event.h"
#include "HAL_Def.h"
// #include "CommonMacro.h"
#include <esp_log.h>
#include "driver/i2c_master.h"

typedef enum {
    SUPER_DIAL_NULL = 0,
    SUPER_DIAL_LEFT = 1,
    SUPER_DIAL_RIGHT= 2,
} SuperDialMotion;

typedef enum {
    HASS_LEFT = 1,
    HASS_RIGHT= 2,
    HASS_PUSH = 3,
    HASS_MAX,
} HassMotion;

#define KNOB_MOTOR_NUM    0
#define ENCODER_MOTOR_NUM 1

enum BOT_RUNNING_MODE {
    BOT_RUNNING_MODE,
    BOT_RUNNING_BALANCE,
};

typedef enum
{
    MOTOR_UNBOUND_FINE_DETENTS,        // Fine values\nWith detents
    MOTOR_UNBOUND_NO_DETENTS,
    MOTOR_SUPER_DIAL, 
    MOTOR_UNBOUND_COARSE_DETENTS, // Coarse values\nStrong detents\n unbound
    MOTOR_BOUND_0_12_NO_DETENTS,
    MOTOR_BOUND_LCD_BK_BRIGHTNESS,
    MOTOR_BOUND_LCD_BK_TIMEOUT,
    MOTOR_COARSE_DETENTS,       // Coarse values\nStrong detents
    MOTOR_FINE_NO_DETENTS,     // Fine values\nNo detents
    MOTOR_ON_OFF_STRONG_DETENTS,             // "On/off\nStrong detent"
    MOTOR_RETURN_TO_CENTER,
    MOTOR_MAX_MODES, //

} MOTOR_RUNNING_MODE_E;

typedef enum {
    IMU_ENCODER_I2C_BUS,
    ENCODER_I2C_BUS,
} I2C_BUS;

#define MOTOR_MAX_SPEED  15
/** using gyro_z */
// #define BOT_MAX_STEERING 500 
/** no feedback */
#define BOT_MAX_STEERING 60


namespace HAL
{
    void Init();

    void motor_init(void);
    // void motor_task(void *pvParameters);
    int get_motor_position(int id);
    void update_motor_mode(int id, int mode , int init_position);
    void motor_shake(int id, int strength, int delay_time);
    void motor_set_speed(float speed, float steering);
    double motor_get_cur_angle(void);
    double get_motor_angle_offset(int id);

    void  imu_init(void);
    void  imu_update(void *pvParameters);
    void  imu_update(void);
    float imu_get_abs_yaw(void);
    float imu_get_pitch(void);
    float imu_get_yaw(void);
    float imu_get_gyro_z(void);

    i2c_master_bus_handle_t get_i2c_bus(I2C_BUS num);
}
#endif