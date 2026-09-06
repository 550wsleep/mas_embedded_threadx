#ifndef _INFANTRYTEST_DEF_H_
#define _INFANTRYTEST_DEF_H_

#include <stdint.h>

#define PITCH_HORIZON_ANGLE 0.0f
#define PITCH_MAX_ANGLE     40.0f
#define PITCH_MIN_ANGLE     -20.0f
#define YAW_CHASSIS_ALIGN_ECD         6020

#define CHASSIS_MAX_SPEED_MPS         0.40f

#pragma pack(1)

typedef enum
{
    gimbal_zero_force = 0,
    gimbal_gyro_mode,
} gimbal_mode_e;

typedef struct
{
    float         yaw;
    float         pitch;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_t;

typedef enum
{
    chassis_zero_force = 0,
    chassis_follow_gimbal_yaw,
    chassis_rotate,
    chassis_rotate_reverse,
} chassis_mode_e;

typedef struct
{
    float          vx;
    float          vy;
    float          wz;
    float          offset_angle;
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_t;

#pragma pack()

#endif