#ifndef _GIMBAL_FUNC_H_
#define _GIMBAL_FUNC_H_

#include "motor_damiao.h"
#include "hero2_def.h"

int     gimbal_init(void);
uint8_t gimbal_func(Gimbal_Ctrl_Cmd_t *cmd, float *yaw_angle_rad);

#endif
