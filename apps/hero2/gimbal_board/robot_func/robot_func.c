#include "robot_func.h"
#include "module_remote.h"
#include <stddef.h>
#include "hero2_def.h"

void RemoteControlSet(Gimbal_Ctrl_Cmd_t *cmd)
{
    if (cmd == NULL) return;

    uint8_t state = Module_Remote_get_offline_status();
    if (!(state & 0x01))
    {
        /* 遥控掉线: 零力 + 命令清零 */
        cmd->gimbal_mode = gimbal_zero_force;
        cmd->vx_cmd      = 0;
        return;
    }

    cmd->vx_cmd        = (int8_t)((float)Module_Remote_get_channel(2) * 10.0f / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS));
    
    int16_t ch6 = Module_Remote_get_channel(6);
    if (ch6 == SBUS_CHX_UP)
    {
        cmd->gimbal_mode = gimbal_remote_mode;
        cmd->yaw_cmd += (float)Module_Remote_get_channel(4) /
                        (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS) * 3.0f * 0.002f; /* 符号实车修 */
    }
    else if (ch6 == SBUS_CHX_DOWN)
    {
        cmd->gimbal_mode = gimbal_zero_force;
        cmd->vx_cmd      = 0;
    }
    
}
