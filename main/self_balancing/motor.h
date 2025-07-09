#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "hal.h"
#include "esp_log.h"
#include "boards/self-balancing-robot/config.h"
#include "driver/spi_master.h"
#include "settings.h"

// #define pi 3.1415926
#define init_smooth 1000 // 该值越大，初始化越慢。以防受到干扰。
#define volt_limit 5.0000

#define XK_INVERT_ROTATION  true

typedef struct {
    bool is_outbound;
    int32_t position;
    double angle_offset;
}MotorStatusInfo;

struct XKnobConfig {
    // 可以运动的个数
    int32_t num_positions;        
    // 位置
    int32_t position;             
    // 位置宽度弧度 或者是每一步的度数
    float position_width_radians; 
    // 正常旋转时的制动强度
    float detent_strength_unit;  
    // 超出界限后的制动强度
    float endstop_strength_unit;  
    // 每一步弧度的放大值
    float snap_point; 
    // 描述符            
    char descriptor[50];          
};


#endif // 