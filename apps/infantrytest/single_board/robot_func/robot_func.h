#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include "infantrytest_def.h"

int16_t CalcOffsetAngle(float getyawangle);

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl);

#endif
