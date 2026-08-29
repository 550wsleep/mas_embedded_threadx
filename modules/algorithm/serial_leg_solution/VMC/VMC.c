/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 12:10:31
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-27 17:34:51
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\VMC\VMC.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "VMC.h"
#include "solution_def.h"
#include "length_control.h"
#include "observer.h"
#include "kinematics.h"
#include "LQR_leg.h"
#include "leg_coor.h"

static VMC VMC_ctrl;

void VMC_calc()
{
    const Length       *le   = Length_get();
    const fk           *ki_L = kinematics_get_left();
    const fk           *ki_R = kinematics_get_right();
    const LQR          *lqr  = LQR_get();
    const coordination *coor = leg_coor_get();
    
    // float F_L = AGC / arm_cos_f32(ki_L->theta) + le->F_L_pid;  
    // float F_R = AGC / arm_cos_f32(ki_R->theta) + le->F_R_pid; //将来要加支持力结算，解决theta摆角过大区域无穷的问题

    float F_L = AGC + le->F_L_pid;  
    float F_R = AGC + le->F_R_pid; 
    
    float Tp_L = lqr->Tpl + coor->Tp_L_coor;
    float Tp_R = lqr->Tpr + coor->Tp_R_coor;

    VMC_ctrl.T_1_L = ki_L->J_11 * F_L + ki_L->J_21 *Tp_L;
    VMC_ctrl.T_2_L = ki_L->J_12 * F_L + ki_L->J_22 *Tp_L;

    VMC_ctrl.T_1_R = ki_R->J_11 * F_R + ki_R->J_21 *Tp_R;
    VMC_ctrl.T_2_R = ki_R->J_12 * F_R + ki_R->J_22 *Tp_R;
}

const VMC *VMC_get()
{
    return &VMC_ctrl;
}
