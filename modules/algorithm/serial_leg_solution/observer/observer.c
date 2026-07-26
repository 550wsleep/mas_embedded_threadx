/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-25 19:41:18
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-07-26 11:17:15
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\observer\observer.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "observer.h"
#include "module_ins.h"
#include "module_bmi088.h"
#include "solution_def.h"
#include "user_lib.h"

static observer obs;  
const Ins_t           *ins = NULL;
const Bmi088_device_t *bmi = NULL;


void observer_init(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[])
{
    ins = Module_INS_get();
    bmi = Module_BMI088_get_device();
    obs.phi_1_L = motor_8009[3]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.phi_4_L = motor_8009[0]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.phi_1_R = motor_8009[2]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.phi_4_R = motor_8009[1]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;//角度还需要加补偿
    obs.x_L     = motor_3508[0]->base.measure.total_angle*WHEEL_R;
    obs.dx_L    = motor_3508[0]->base.measure.angular_velocity*WHEEL_R;
    obs.x_R     = motor_3508[1]->base.measure.total_angle*WHEEL_R;
    obs.dx_R    = motor_3508[1]->base.measure.angular_velocity*WHEEL_R;
    obs.pitch   = ins->euler_rad[1];
    obs.dpitch  = bmi->gyro[0];
    obs.roll    = ins->euler_rad[0];
    obs.yaw     = ins->euler_rad[2];
    obs.initialized = 1;
}

void observer_update(void)
{
   
}

const observer *observer_get(void)
{
    if (obs.initialized)
        return &obs;
    return NULL;
}
