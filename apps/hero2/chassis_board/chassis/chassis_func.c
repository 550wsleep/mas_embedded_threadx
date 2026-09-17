/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-08-28 21:08:45
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-09-10 20:02:16
 * @FilePath: \mas_embedded_threadx\apps\hero2\chassis_board\chassis\chassis_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-08-28 21:08:45
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-08-29 13:40:56
 * @FilePath: \mas_embedded_threadx\apps\hero2\chassis_board\chassis\chassis_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Description: 串腿应用层 - 4×DM8009(髋关节) + 2×M3508(轮毂)
 *
 * ========== DM8009 髋关节电机 (编号 1-4, CAN1, MIT模式) ==========
 *   # | 变量  | 位置 | 安装 | 反转 |  CAN ID  | timeout_ms
 *   --|-------|------|----------|------|------|------|----------|------------
 *   1 | hip_1 | 左前 | 正装 |  N   | 0x01/0x11 | 1000
 *   2 | hip_2 | 右前 | 反装 |  Y   | 0x02/0x12 | 1000
 *   3 | hip_3 | 右后 | 反装 |  Y   | 0x03/0x13 | 1000
 *   4 | hip_4 | 左后 | 正装 |  N   | 0x04/0x14 | 1000
 *
 *   joints[] 顺序: {hip_4, hip_3, hip_2, hip_1}  (左后→右后→右前→左前)
 *   VMC 映射:   [0]hip_4→T_2_L  [1]hip_3→T_2_R  [2]hip_2→T_1_R  [3]hip_1→T_1_L
 *
 * ========== M3508 轮毂电机 (CAN1) ==========
 *   # | 变量    | 位置 | 安装 | 反转 | CAN ID
 *   --|---------|------|------|------|--------
 *   5 | wheel_l | 左轮 | 正装 |  N   | 1
 *   6 | wheel_r | 右轮 | 反装 |  Y   | 4
 *
 * ========== 控制流水线 (2ms) ==========
 * observer_update → kinematics_calc → length_control
 * → LQR_calc → leg_coor_control → roll_control → gas_spring_calc → VMC_calc
 *
 * ========== 安全机制 ==========
 * 遥控掉线 或 任意电机离线 → 全部 Stop
 */
#include "chassis_func.h"
#include "module_offline.h"
#include "module_remote.h"
#include "motor_def.h"
#include "motor_damiao.h"
#include "motor_dji.h"
#include "bsp_dwt.h"
#include "user_lib.h"

#include "observer.h"
#include "kinematics.h"
#include "length_control.h"
#include "LQR_leg.h"
#include "leg_coor.h"
#include "roll_control.h"
#include "gas_spring.h"
#include "VMC.h"

#include <stdint.h>


static DM_Motor_t *hip_1, *hip_2, *hip_3, *hip_4;
static DJI_Motor_t *wheel_l, *wheel_r;

void chassis_init(void)
{
    /* ========== DM8009 髋关节电机 x4 (CAN1, MIT模式) ========== */
    Motor_Init_Config_s dm_config = {
        .offline_init_config = {
            .name       = "dm8009",
            .beep_times = 1,
            .timeout_ms = 1000,
            .enable     = 1,
        },
        .setting_init_config = {
            .loop_type  = OPEN_LOOP,
            .enableflag = 1,
        },
        .motor_init_info = {
            .motor_type = DM8009,
            .max_torque = 54.0f,   /* 对齐字段量程（luntui1 J8009 ±54）；OPEN_LOOP 下为死配置，仅作语义标注 */
        },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config = {
            .can = { .hcan = BSP_CAN_HANDLE1 },
        },
    };

    /* 左前 hip_1 (正装) */
    dm_config.transport_config.can.tx_id = 0x01;
    dm_config.transport_config.can.rx_id = 0x11;
    dm_config.offline_init_config.name = "hip_1";
    dm_config.offline_init_config.beep_times = 1;
    dm_config.setting_init_config.motor_reverse_flag = 0;
    hip_1 = Motor_DM_Init(&dm_config, DM_MIT_MODE);

    /* 右前 hip_2 (反装) */
    dm_config.transport_config.can.tx_id = 0x02;
    dm_config.transport_config.can.rx_id = 0x12;
    dm_config.offline_init_config.name = "hip_2";
    dm_config.offline_init_config.beep_times = 2;
    dm_config.setting_init_config.motor_reverse_flag = 1;
    hip_2 = Motor_DM_Init(&dm_config, DM_MIT_MODE);

    /* 右后 hip_3 (反装) */
    dm_config.transport_config.can.tx_id = 0x03;
    dm_config.transport_config.can.rx_id = 0x13;
    dm_config.offline_init_config.name = "hip_3";
    dm_config.offline_init_config.beep_times = 3;
    dm_config.setting_init_config.motor_reverse_flag = 1;
    hip_3 = Motor_DM_Init(&dm_config, DM_MIT_MODE);

    /* 左后 hip_4 (正装) */
    dm_config.transport_config.can.tx_id = 0x04;
    dm_config.transport_config.can.rx_id = 0x14;
    dm_config.offline_init_config.name = "hip_4";
    dm_config.offline_init_config.beep_times = 4;
    dm_config.setting_init_config.motor_reverse_flag = 0;
    hip_4 = Motor_DM_Init(&dm_config, DM_MIT_MODE);

    /* ========== M3508 轮毂电机 x2 (CAN1, 开环) ========== */
    Motor_Init_Config_s dji_config = {
        .offline_init_config = {
            .name       = "m3508",
            .beep_times = 1,
            .timeout_ms = 1000,
            .enable     = 1,
        },
        .setting_init_config = {
            .loop_type  = OPEN_LOOP,
            .enableflag = 1,
        },
        .motor_init_info = {
            .motor_type      = M3508,
            .max_torque      = 5.32f,
            .gear_ratio      = 16,
            .torque_constant = 0.016f,
        },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config = {
            .can = { .hcan = BSP_CAN_HANDLE1 },
        },
    };

    /* 左轮 wheel_l (正装) */
    dji_config.transport_config.can.tx_id = 1;
    dji_config.offline_init_config.name = "wheel_l";
    dji_config.offline_init_config.beep_times = 5;
    dji_config.setting_init_config.motor_reverse_flag = 0;
    wheel_l = Motor_DJI_Init(&dji_config);

    /* 右轮 wheel_r (反装, CAN ID=4, 与 luntui1 硬件一致: 电调拨码4, 0x200帧data[6-7]) */
    dji_config.transport_config.can.tx_id = 4;
    dji_config.offline_init_config.name = "wheel_r";
    dji_config.offline_init_config.beep_times = 6;
    dji_config.setting_init_config.motor_reverse_flag = 1;
    wheel_r = Motor_DJI_Init(&dji_config);

    /* ========== 串腿算法模块初始化 ========== */
    DM_Motor_t  *joint_arr[4] = {hip_4, hip_3, hip_2, hip_1};
    DJI_Motor_t *wheel_arr[2] = {wheel_l, wheel_r};
    observer_init(joint_arr, wheel_arr);
    kinematics_init();
    length_init();
    leg_coor_init();
    roll_init();
    gas_spring_init();
}

void chassis_func(void)
{
    /* ---- 安全检测 ---- */
    uint8_t remote_status = Module_Remote_get_offline_status();
    bool remote_online = (remote_status != 0 && remote_status != 2);

    DM_Motor_t  *joints[4] = {hip_4, hip_3, hip_2, hip_1};
    DJI_Motor_t *wheels[2] = {wheel_l, wheel_r};

    bool all_online = true;
    for (int i = 0; i < 4; i++) {
        if (joints[i] == NULL ||
            Module_Offline_get_device_status(joints[i]->base.offline_dev)) {
            all_online = false;
            break;
        }
    }
    for (int i = 0; i < 2; i++) {
        if (wheels[i] == NULL ||
            Module_Offline_get_device_status(wheels[i]->base.offline_dev)) {
            all_online = false;
            break;
        }
    }

    if (!remote_online || !all_online) {
        for (int i = 0; i < 4; i++) {
            if (joints[i] != NULL)
                Motor_Stop((Motor_Base *)joints[i]);
        }
        for (int i = 0; i < 2; i++) {
            if (wheels[i] != NULL)
                Motor_Stop((Motor_Base *)wheels[i]);
        }
        return;
    }

    /* ---- 掉线恢复后重新使能 (Motor_Stop 清 enableflag, 必须恢复, 对齐 sentry 范式) ---- */
    for (int i = 0; i < 4; i++) {
        if (joints[i] != NULL)
            Motor_Start((Motor_Base *)joints[i]);
    }
    for (int i = 0; i < 2; i++) {
        if (wheels[i] != NULL)
            Motor_Start((Motor_Base *)wheels[i]);
    }

    /* ---- 控制流水线 ---- */
    BSP_DWT_Delay(0.0002f); /* 200us */
    observer_update(joints, wheels);
    kinematics_calc();
    length_control();
    LQR_calc(0.0f, 0.0f);
    leg_coor_control();
    roll_control();
    gas_spring_calc();
    VMC_calc();

    /* ---- VMC力矩输出 x 5% ---- */
    const VMC *vmc = VMC_get();
    if (vmc != NULL) {
        //Motor_SetOutputTorque((Motor_Base *)hip_4,  vmc->T_2_L);
        //Motor_SetOutputTorque((Motor_Base *)hip_3, -vmc->T_2_R);
        //延时200um
        //Motor_SetOutputTorque((Motor_Base *)hip_2, -vmc->T_1_R);
        //Motor_SetOutputTorque((Motor_Base *)hip_1,  vmc->T_1_L);
    }
    const LQR *lqr = LQR_get();
    if (lqr != NULL) {
        //Motor_SetOutputTorque((Motor_Base *)wheel_l,  lqr->TL);
        //Motor_SetOutputTorque((Motor_Base *)wheel_r, -lqr->TR);
        Motor_SetOutputTorque((Motor_Base *)wheel_l,  0.5f);
        Motor_SetOutputTorque((Motor_Base *)wheel_r, -0.5f);
    }
}

