/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 20:23:17
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-26 20:52:03
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\leg_length\length_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "length_control.h"
#include "solution_def.h"
#include "kinematics.h"
#include "pid.h"

static Length lenth_ctrl;
static PIDInstance pid_left;
static PIDInstance pid_right;

void length_init()
{
    lenth_ctrl.target_length_L = 0.2f;
    lenth_ctrl.target_length_R = 0.2f;

    PID_Init_Config_s pd_config = {
        .Kp       = 1.0f,   // 比例系数（按实际调）
        .Ki       = 0.0f,     // I=0 就是 PD
        .Kd       = 0.01f,    // 微分系数（按实际调）
        .MaxOut   = 1.0f,    // 输出限幅 N
        .DeadBand = 0.0f,   // 死区 m
        .Improve  = PID_Derivative_On_Measurement,  // 微分在测量值上算，防突变
    };

    PIDInit(&pid_left,  &pd_config);
    PIDInit(&pid_right, &pd_config);
}

void length_control ()
{
    const fk *fk_left = kinematics_get_left();
    const fk *fk_right = kinematics_get_right();
    
    lenth_ctrl.F_L_pid = PIDCalculate(&pid_left,  fk_left->L_0,  lenth_ctrl.target_length_L);
    lenth_ctrl.F_R_pid = PIDCalculate(&pid_right, fk_right->L_0, lenth_ctrl.target_length_R); 
}

const Length *Length_get()
{
    return &lenth_ctrl;
}
