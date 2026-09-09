/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-27 13:35:45
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-27 16:02:41
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\roll_comp\roll_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "roll_control.h"
#include "solution_def.h"
#include "observer.h"

static roll_compf roll_ctrl;
static PIDInstance  roll_pid;

void roll_init(void)
{
    PID_Init_Config_s config = {
        .Kp         = 15.0f,
        .Ki         = 0.0f,
        .Kd         = 20.0f,
        .MaxOut     = 25.0f,
        .DeadBand   = 0.0f,
        .Improve    = PID_Derivative_On_Measurement,
        .IntegralLimit = 0.0f,
    };
    PIDInit(&roll_pid, &config);
}


void roll_control()
{
    const observer *obs = observer_get();

    float output = PIDCalculate(&roll_pid, 0.0f, obs->roll);

    roll_ctrl.F_L_roll =  output;
    roll_ctrl.F_R_roll = -output;
}


const roll_compf *roll_comf_get()
{
    return &roll_ctrl;
}