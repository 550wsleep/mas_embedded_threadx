/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-08-30 14:51:09
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-08-30 14:51:12
 * @FilePath: \RM_note\_temp_chassis.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "chassis_func.h"
#include "module_offline.h"
#include "motor_def.h"
#include "motor_dji.h"
#include "user_lib.h"
#include <stdint.h>
#include "chassis_type.h"

#define LOG_TAG "app_chassis"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

static DJI_Motor_t                *chassis_motors[4];
static float                       chassis_vx, chassis_vy, chassis_wz;
static PIDInstance                 chassis_follow_pid;
static const Chassis_Diff_Config_s chassis_diff_config = {
    .decele_ratio = 16.0f,
    .wheel_base_x = 0.5f,
    .wheel_base_y = 0.3f,
    .wheel_radius = 0.075f,
};

void chassis_init(void)
{
    PID_Init_Config_s config = {.MaxOut = 2, .IntegralLimit = 0.01, .DeadBand = 10, .Kp = 0.1, .Ki = 0, .Kd = 0.001, .Improve = 0x01};
    PIDInit(&chassis_follow_pid, &config);

    Motor_Init_Config_s chassis_motor_config = {
        .offline_init_config    = {.timeout_ms = 100, .enable = 1},
        .transport              = MOTOR_TRANSPORT_CAN,
        .transport_config       = {.can.hcan = BSP_CAN_HANDLE1},
        .controller_init_config = {.lqr_init = {.K = {0.008f}, .state_dim = 1}},
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = SPEED_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_LQR,
            },
        .motor_init_info = {.motor_type = M3508, .gear_ratio = 16, .max_torque = 5.32, .torque_constant = 0.016},
    };

    // motors[0] LF: tx_id=4, reverse=1
    chassis_motor_config.transport_config.can.tx_id             = 4;
    chassis_motor_config.setting_init_config.motor_reverse_flag = 1;
    chassis_motor_config.offline_init_config.name               = "m3508_4";
    chassis_motor_config.offline_init_config.beep_times         = 4;
    chassis_motors[0]                                           = Motor_DJI_Init(&chassis_motor_config);
    if (chassis_motors[0] == NULL)
    {
        LOG_E("chassis motor[0] init failed");
        return;
    }

    // motors[1] LB: tx_id=1, reverse=1
    chassis_motor_config.transport_config.can.tx_id             = 1;
    chassis_motor_config.setting_init_config.motor_reverse_flag = 1;
    chassis_motor_config.offline_init_config.name               = "m3508_1";
    chassis_motor_config.offline_init_config.beep_times         = 1;
    chassis_motors[1]                                           = Motor_DJI_Init(&chassis_motor_config);
    if (chassis_motors[1] == NULL)
    {
        LOG_E("chassis motor[1] init failed");
        return;
    }

    // motors[2] RB: tx_id=2, reverse=0
    chassis_motor_config.transport_config.can.tx_id             = 2;
    chassis_motor_config.setting_init_config.motor_reverse_flag = 0;
    chassis_motor_config.offline_init_config.name               = "m3508_2";
    chassis_motor_config.offline_init_config.beep_times         = 2;
    chassis_motors[2]                                           = Motor_DJI_Init(&chassis_motor_config);
    if (chassis_motors[2] == NULL)
    {
        LOG_E("chassis motor[2] init failed");
        return;
    }

    // motors[3] RF: tx_id=3, reverse=0
    chassis_motor_config.transport_config.can.tx_id             = 3;
    chassis_motor_config.setting_init_config.motor_reverse_flag = 0;
    chassis_motor_config.offline_init_config.name               = "m3508_3";
    chassis_motor_config.offline_init_config.beep_times         = 3;
    chassis_motors[3]                                           = Motor_DJI_Init(&chassis_motor_config);
    if (chassis_motors[3] == NULL)
    {
        LOG_E("chassis motor[3] init failed");
        return;
    }

    LOG_I("Chassis initialized");
}

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd)
{
    if (chassis_cmd != NULL)
    {
        if (!Module_Offline_get_device_status(chassis_motors[0]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[1]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[2]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[3]->base.offline_dev))
        {
            if (chassis_cmd->chassis_mode == chassis_zero_force)
            {
                Motor_Stop((Motor_Base *)chassis_motors[0]);
                Motor_Stop((Motor_Base *)chassis_motors[1]);
                Motor_Stop((Motor_Base *)chassis_motors[2]);
                Motor_Stop((Motor_Base *)chassis_motors[3]);
            }
            else
            {
                Motor_Start((Motor_Base *)chassis_motors[0]);
                Motor_Start((Motor_Base *)chassis_motors[1]);
                Motor_Start((Motor_Base *)chassis_motors[2]);
                Motor_Start((Motor_Base *)chassis_motors[3]);

                switch (chassis_cmd->chassis_mode)
                {
                case chassis_rotate_reverse:
                    chassis_wz = -1.5;
                    break;
                case chassis_follow_gimbal_yaw:
                    {
                        float offset_angle = chassis_cmd->offset_angle;
                        PIDCalculate(&chassis_follow_pid, -offset_angle, 0);
                        chassis_wz = chassis_follow_pid.Output;
                    }
                    break;
                case chassis_rotate:
                    chassis_wz = 1.5;
                    break;
                default:
                    break;
                }

                chassis_vx = chassis_cmd->vx;
                chassis_vy = chassis_cmd->vy;

                float cos_theta = cosf(chassis_cmd->offset_angle * DEGREE_2_RAD);
                float sin_theta = sinf(chassis_cmd->offset_angle * DEGREE_2_RAD);

                float chassis_vx_trans = chassis_vx * cos_theta - chassis_vy * sin_theta;
                float chassis_vy_trans = chassis_vx * sin_theta + chassis_vy * cos_theta;

                Chassis_Mecanum_Calc(chassis_motors, &chassis_diff_config, chassis_vx_trans, chassis_vy_trans, chassis_wz);
            }
        }
        else
        {
            Motor_Stop((Motor_Base *)chassis_motors[0]);
            Motor_Stop((Motor_Base *)chassis_motors[1]);
            Motor_Stop((Motor_Base *)chassis_motors[2]);
            Motor_Stop((Motor_Base *)chassis_motors[3]);
        }
    }
}