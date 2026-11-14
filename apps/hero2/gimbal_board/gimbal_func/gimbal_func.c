/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-11-12 19:49:41
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-11-14 11:15:32
 * @FilePath: \mas_embedded_threadx\apps\hero2\gimbal_board\gimbal_func\gimbal_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "gimbal_func.h"
#include "motor_damiao.h"
#include "module_ins.h"
#include "module_bmi088.h"
#include "hero2_def.h"


static DM_Motor_t *yaw_motor = NULL;
static const Ins_t           *ins       = NULL;      /* ← 新增 */
static const Bmi088_device_t *bmi088    = NULL;      /* ← 新增 */


int gimbal_init(void)
{
    ins    = Module_INS_get();                       /* ← 新增, 判 NULL */
    bmi088 = Module_BMI088_get_device();
    Motor_Init_Config_s yaw_motor_config = {
        .offline_init_config = {.name = "dm4310_yaw", .timeout_ms = 100, .beep_times = 2, .enable = 1},
        .transport           = MOTOR_TRANSPORT_CAN,
        .transport_config.can = {
            .hcan  = BSP_CAN_HANDLE2,   /* CAN2: 板间 + yaw + 拨盘 */
            .tx_id = 0x01,          
            .rx_id = 0x11,            
        },
        .controller_init_config = {
            .other_angle_feedback_ptr = &ins->YawTotalAngle_rad,   /* 外部角度反馈 */
            .other_speed_feedback_ptr = &bmi088->gyro[2],          /* yaw 轴角速度 */
            .lqr_init = { .K = {5.47f, 0.56f}, .state_dim = 2 },   /* 起步 K, 实车再调 */
        },
        .setting_init_config = {
            .algorithm_type        = CONTROL_LQR,
            .angle_feedback_source = 1,   /* 1 = 外部 IMU */
            .speed_feedback_source = 1,   /* 1 = 外部 IMU */
            .loop_type             = ANGLE_LOOP,
        },
        .motor_init_info = { .motor_type = DM4310, .max_torque = 10.0f },
    };
    yaw_motor = Motor_DM_Init(&yaw_motor_config,DM_MIT_MODE);
    if (yaw_motor == NULL) return -1;
    return 0;
}

uint8_t gimbal_func(Gimbal_Ctrl_Cmd_t *cmd, float *yaw_angle_rad)
{
    if (yaw_motor == NULL || cmd == NULL) return 0;

    if (cmd->gimbal_mode == gimbal_zero_force)
        Motor_Stop((Motor_Base *)yaw_motor);                 /* ← 零力分支 */
    else
    {
        Motor_Start((Motor_Base *)yaw_motor);
        Motor_SetRef((Motor_Base *)yaw_motor, cmd->yaw_cmd); /* ← yaw_cmd */
    }

    if (yaw_angle_rad != NULL)
        *yaw_angle_rad = yaw_motor->base.measure.single_round_angle;  /* ← 编码器带出去 */
    return Module_Offline_get_device_status(yaw_motor->base.offline_dev) ? 0 : 1;  /* ← 掉线返回 0 */
}
