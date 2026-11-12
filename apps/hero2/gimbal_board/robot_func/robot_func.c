#include "robot_func.h"
#include "module_remote.h"
#include <stddef.h>
#include "hero2_def.h"

void RemoteControlSet(Gimbal_Ctrl_Cmd_t *cmd)
{
    if (cmd == NULL) return;

    cmd->body_target_dx_gb = (int8_t)((float)Module_Remote_get_channel(2) * 10.0f / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS));
    cmd->turn_cmd_gb       = (int8_t)((float)Module_Remote_get_channel(4) * 10.0f / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS));
}
