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
        .Kp         = 500.0f,   // 比例（按实际调）
        .Ki         = 0.0f,     // I=0 → PD
        .Kd         = 20.0f,    // 微分
        .MaxOut     = 50.0f,    // ← 输出范围 -50~50
        .DeadBand   = 0.01f,    // ← 误差死区 ±0.01
        .Improve    = PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 0.0f,  // I=0 就不用限了
    };
    PIDInit(&roll_pid, &config);
}


void roll_control()
{
    const observer *obs = observer_get();
    
    float output = PIDCalculate(&roll_pid, obs->roll, 0.0f);

    // 左腿推力补偿 = PID输出
    roll_ctrl.F_L_roll =  output;   // 左腿补偿
    roll_ctrl.F_R_roll = -output;   // 右腿反向
}


const roll_compf *roll_comf_get()
{
    return &roll_ctrl;
}


