/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-08-28 21:08:45
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-08-29 14:54:40
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
 * @Author: 550wsleep
 * @Date: 2026-07-24
 * @FilePath: \mas_embedded_threadx\apps\hero2\chassis_board\chassis\chassis_func.c
 * @Description: 串腿应用层 - 4×DM8009(髋/膝) + 2×M3508(轮毂)
 *
 * ========== DM8009 关节电机配置 ==========
 * ┌──────────────┬──────────┬──────────┬──────────┬──────────┐
 * │ 配置项        │ 左上髋    │ 右上髋    │ 右下膝    │ 左下膝    │
 * │              │ zuo_kuan │ you_kuan │ you_xi   │ zuo_xi   │
 * ├──────────────┼──────────┼──────────┼──────────┼──────────┤
 * │ 离线检测名称  │ zuo_kuan │ you_kuan │ you_xi   │ zuo_xi   │
 * │ 离线蜂鸣次数  │ 1        │ 2        │ 3        │ 4        │
 * │ 离线超时(ms) │ 100      │ 100      │ 100      │ 100      │
 * │ 离线检测使能  │ 开启     │ 开启     │ 开启     │ 开启     │
 * │ 闭环类型      │ 开环     │ 开环     │ 开环     │ 开环     │
 * │ 电机使能      │ 开启     │ 开启     │ 开启     │ 开启     │
 * │ 电机反转      │ 不反转   │ 不反转   │ 不反转   │ 不反转   │
 * │ 反馈反转      │ 不反转   │ 不反转   │ 不反转   │ 不反转   │
 * │ 角度反馈源    │ 电机反馈 │ 电机反馈 │ 电机反馈 │ 电机反馈 │
 * │ 速度反馈源    │ 电机反馈 │ 电机反馈 │ 电机反馈 │ 电机反馈 │
 * │ 控制算法      │ PID      │ PID      │ PID      │ PID      │
 * │ 电机型号      │ DM8009   │ DM8009   │ DM8009   │ DM8009   │
 * │ 最大力矩(Nm)  │ 30       │ 30       │ 30       │ 30       │
 * │ 减速比        │ 1        │ 1        │ 1        │ 1        │
 * │ 力矩常数      │ 1        │ 1        │ 1        │ 1        │
 * │ 传输层        │ CAN      │ CAN      │ CAN      │ CAN      │
 * │ CAN句柄       │ CAN1     │ CAN1     │ CAN1     │ CAN1     │
 * │ 发送ID(tx)   │ 0x01     │ 0x02     │ 0x03     │ 0x04     │
 * │ 接收ID(rx)   │ 0x11     │ 0x12     │ 0x13     │ 0x14     │
 * │ 工作模式      │ MIT      │ MIT      │ MIT      │ MIT      │
 * └──────────────┴──────────┴──────────┴──────────┴──────────┘
 * 注意: rx_id > tx_id, 否则初始化失败
 *
 * ========== M3508 轮毂电机配置 ==========
 * ┌──────────────┬──────────┬──────────┐
 * │ 配置项        │ 左轮毂    │ 右轮毂    │
 * │              │ zuo_lun  │ you_lun  │
 * ├──────────────┼──────────┼──────────┤
 * │ 离线检测名称  │ zuo_lun  │ you_lun  │
 * │ 离线蜂鸣次数  │ 5        │ 6        │
 * │ 离线超时(ms) │ 100      │ 100      │
 * │ 离线检测使能  │ 开启     │ 开启     │
 * │ 闭环类型      │ 开环     │ 开环     │
 * │ 电机使能      │ 开启     │ 开启     │
 * │ 电机反转      │ 不反转   │ 不反转   │
 * │ 反馈反转      │ 不反转   │ 不反转   │
 * │ 角度反馈源    │ 电机反馈 │ 电机反馈 │
 * │ 速度反馈源    │ 电机反馈 │ 电机反馈 │
 * │ 控制算法      │ PID      │ PID      │
 * │ 电机型号      │ M3508    │ M3508    │
 * │ 最大力矩(Nm)  │ 5.32     │ 5.32     │
 * │ 减速比        │ 16       │ 16       │
 * │ 力矩常数      │ 0.016    │ 0.016    │
 * │ 传输层        │ CAN      │ CAN      │
 * │ CAN句柄       │ CAN2     │ CAN2     │
 * │ 发送ID(tx)   │ 1        │ 2        │
 * │ 接收ID(rx)   │ 自动     │ 自动     │
 * │ 工作模式      │ -        │ -        │
 * └──────────────┴──────────┴──────────┘
 * 注意: rx_id 由 MotorSenderGrouping 自动分配, 不用填
 *
 * ========== 对应代码结构体字段 ==========
 * 离线检测名称/蜂鸣/超时/使能 → offline_init_config
 * 闭环类型/使能/反转/反馈源/算法 → setting_init_config
 * 型号/力矩/减速比/力矩常数     → motor_init_info
 * 传输层/CAN句柄/发送ID/接收ID  → transport + transport_config.can
 * 工作模式                      → Motor_DM_Init 第二个参数
 *
 * ========== 控制流水线 (2ms) ==========
 * observer_update → kinematics_calc → length_control
 * → LQR_calc → leg_coor_control → roll_control → VMC_calc
 * → SetForwardTorque × TORQUE_TEST_SCALE(5%)
 *
 * ========== 安全机制 ==========
 * 遥控掉线 或 任意电机离线 → 全部 Stop
 * (Stop 即 enableflag=0, motor线程自动发零值)
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
#include "VMC.h"

#include <stdint.h>

#define TORQUE_TEST_SCALE 0.05f

static DM_Motor_t *zuo_kuan, *you_kuan, *zuo_xi, *you_xi;
static DJI_Motor_t *zuo_lun, *you_lun;

void chassis_init(void)
{
    /* ========== DM8009 关节电机 x4 (CAN1, MIT模式) ========== */
    Motor_Init_Config_s dm_config = {
        .offline_init_config = {
            .name       = "dm8009",
            .beep_times = 1,
            .timeout_ms = 100,
            .enable     = 1,
        },
        .setting_init_config = {
            .loop_type  = OPEN_LOOP,
            .enableflag = 1,
        },
        .motor_init_info = {
            .motor_type = DM8009,
            .max_torque = 30.0f,
        },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config = {
            .can = { .hcan = BSP_CAN_HANDLE1 },
        },
    };

    /* 左上髋 */
    dm_config.transport_config.can.tx_id = 0x01;
    dm_config.transport_config.can.rx_id = 0x11;
    dm_config.offline_init_config.name = "zuo_kuan";
    dm_config.offline_init_config.beep_times = 1;
    zuo_kuan = Motor_DM_Init(&dm_config, DM_MIT_MODE);

    /* 右上髋 */
    dm_config.transport_config.can.tx_id = 0x02;
    dm_config.transport_config.can.rx_id = 0x12;
    dm_config.offline_init_config.name = "you_kuan";
    dm_config.offline_init_config.beep_times = 2;
    you_kuan = Motor_DM_Init(&dm_config, DM_MIT_MODE);

    /* 右下膝 */
    dm_config.transport_config.can.tx_id = 0x03;
    dm_config.transport_config.can.rx_id = 0x13;
    dm_config.offline_init_config.name = "you_xi";
    dm_config.offline_init_config.beep_times = 3;
    you_xi = Motor_DM_Init(&dm_config, DM_MIT_MODE);

    /* 左下膝 */
    dm_config.transport_config.can.tx_id = 0x04;
    dm_config.transport_config.can.rx_id = 0x14;
    dm_config.offline_init_config.name = "zuo_xi";
    dm_config.offline_init_config.beep_times = 4;
    zuo_xi = Motor_DM_Init(&dm_config, DM_MIT_MODE);

    /* ========== M3508 轮毂电机 x2 (CAN2, 开环) ========== */
    Motor_Init_Config_s dji_config = {
        .offline_init_config = {
            .name       = "m3508",
            .beep_times = 1,
            .timeout_ms = 100,
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
            .can = { .hcan = BSP_CAN_HANDLE2 },
        },
    };

    /* 左轮 */
    dji_config.transport_config.can.tx_id = 1;
    dji_config.offline_init_config.name = "zuo_lun";
    dji_config.offline_init_config.beep_times = 5;
    zuo_lun = Motor_DJI_Init(&dji_config);

    /* 右轮 */
    dji_config.transport_config.can.tx_id = 2;
    dji_config.offline_init_config.name = "you_lun";
    dji_config.offline_init_config.beep_times = 6;
    you_lun = Motor_DJI_Init(&dji_config);

    /* ========== 串腿算法模块初始化 ========== */
    DM_Motor_t  *joint_arr[4] = {zuo_kuan, you_kuan, you_xi, zuo_xi};
    DJI_Motor_t *wheel_arr[2] = {zuo_lun, you_lun};
    observer_init(joint_arr, wheel_arr);
    kinematics_init();
    length_init();
    leg_coor_init();
    roll_init();
}

void chassis_func(void)
{
    /* ---- 安全检测 ---- */
    uint8_t remote_status = Module_Remote_get_offline_status();
    bool remote_online = (remote_status != 0 && remote_status != 2);

    DM_Motor_t  *joints[4] = {zuo_kuan, you_kuan, you_xi, zuo_xi};
    DJI_Motor_t *wheels[2] = {zuo_lun, you_lun};

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

    /* ---- 控制流水线 ---- */
    BSP_DWT_Delay(0.0002f); /* 200us */
    observer_update(joints, wheels);
    kinematics_calc();
    length_control();
    LQR_calc(0.0f, 0.0f);
    leg_coor_control();
    roll_control();
    VMC_calc();

    /* ---- VMC力矩输出 x 5% ---- */
    const VMC *vmc = VMC_get();
    if (vmc != NULL) {
        Motor_SetForwardTorque((Motor_Base *)zuo_kuan, vmc->T_1_L * TORQUE_TEST_SCALE);
        Motor_SetForwardTorque((Motor_Base *)you_kuan, vmc->T_1_R * TORQUE_TEST_SCALE);
        //延时200um
        Motor_SetForwardTorque((Motor_Base *)you_xi,   vmc->T_2_R * TORQUE_TEST_SCALE);
        Motor_SetForwardTorque((Motor_Base *)zuo_xi,   vmc->T_2_L * TORQUE_TEST_SCALE);
    }
}