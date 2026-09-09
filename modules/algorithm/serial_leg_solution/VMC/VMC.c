/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 12:10:31
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-09-05 10:00:00
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\VMC\VMC.c
 * @Description: VMC虚功解算
 */
#include "VMC.h"
#include "solution_def.h"
#include "length_control.h"
#include "observer.h"
#include "kinematics.h"
#include "LQR_leg.h"
#include "leg_coor.h"
#include "gas_spring.h"
#include "roll_control.h"
#include "arm_math.h"

static VMC VMC_ctrl;

void VMC_calc()
{
    const Length       *le   = Length_get();
    const fk           *ki_L = kinematics_get_left();
    const fk           *ki_R = kinematics_get_right();
    const LQR          *lqr  = LQR_get();
    const coordination *coor = leg_coor_get();
    const GasSpring    *gs   = gas_spring_get();
    const roll_compf   *rc   = roll_comf_get();

    float F_L = ROBOT_MASS * 9.8f / 2.0f / arm_cos_f32(ki_L->theta) - rc->F_L_roll + le->F_L_pid - gs->Fs_L;
    float F_R = ROBOT_MASS * 9.8f / 2.0f / arm_cos_f32(ki_R->theta) - rc->F_R_roll + le->F_R_pid - gs->Fs_R;

    float Tp_L = lqr->Tpl + coor->Tp_L_coor;
    float Tp_R = lqr->Tpr + coor->Tp_R_coor;

    VMC_ctrl.T_1_L = ki_L->J_11 * F_L + ki_L->J_21 * Tp_L;
    VMC_ctrl.T_2_L = ki_L->J_12 * F_L + ki_L->J_22 * Tp_L;

    VMC_ctrl.T_1_R = ki_R->J_11 * F_R + ki_R->J_21 * Tp_R;
    VMC_ctrl.T_2_R = ki_R->J_12 * F_R + ki_R->J_22 * Tp_R;
}

const VMC *VMC_get()
{
    return &VMC_ctrl;
}
