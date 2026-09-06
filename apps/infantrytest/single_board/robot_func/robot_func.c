#include "robot_func.h"
#include "module_remote.h"
#include "user_lib.h"
#include <stdint.h>
#include <string.h>

int16_t CalcOffsetAngle(float getyawangle)
{
    float offset_ecd;
    const float ECD_MAX  = 8191.0f;
    const float ECD_HALF = 4095.5f;

    offset_ecd = getyawangle - YAW_CHASSIS_ALIGN_ECD;

    while (offset_ecd > ECD_HALF) offset_ecd -= ECD_MAX;
    while (offset_ecd < -ECD_HALF) offset_ecd += ECD_MAX;

    return (int16_t)offset_ecd;
}

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl)
{
    if (!Chassis_Ctrl || !Gimbal_Ctrl) return;

    uint8_t state = Module_Remote_get_offline_status();

    if (state & 0x01)
    {
        Chassis_Ctrl->vx =  (float)Module_Remote_get_channel(2) / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);
        Chassis_Ctrl->vy = -(float)Module_Remote_get_channel(1) / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);

        int16_t ch8 = Module_Remote_get_channel(8);
        if (ch8 < 600)
            Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
        else if (ch8 > 1400)
            Chassis_Ctrl->chassis_mode = chassis_rotate_reverse;
        else
            Chassis_Ctrl->chassis_mode = chassis_rotate;

        Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
        Gimbal_Ctrl->yaw   -= 0.001f * (float)Module_Remote_get_channel(4);
        Gimbal_Ctrl->pitch += 0.001f * (float)Module_Remote_get_channel(3);
        VAL_LIMIT(Gimbal_Ctrl->pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
    }
    else
    {
        Chassis_Ctrl->chassis_mode = chassis_zero_force;
        Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
        memset(Chassis_Ctrl, 0, sizeof(Chassis_Ctrl_Cmd_t));
        memset(Gimbal_Ctrl, 0, sizeof(Gimbal_Ctrl_Cmd_t));
    }
}