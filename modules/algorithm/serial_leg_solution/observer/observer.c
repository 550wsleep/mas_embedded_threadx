/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-25 19:41:18
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-09-01 17:20:23
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\observer\observer.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "observer.h"
#include "module_ins.h"
#include "module_bmi088.h"
#include "solution_def.h"
#include "user_lib.h"
#include "motor_damiao.h"
#include "motor_dji.h"

static observer obs;  
const Ins_t           *ins = NULL;
const Bmi088_device_t *bmi = NULL;


void observer_init(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[])
{
    ins = Module_INS_get();
    bmi = Module_BMI088_get_device();
    obs.phi_1_L = motor_8009[3]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_L = motor_8009[3]->base.measure.speed_rad;
    obs.phi_4_L = motor_8009[0]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_L = motor_8009[0]->base.measure.speed_rad;
    obs.phi_1_R = motor_8009[2]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_R = motor_8009[2]->base.measure.speed_rad;
    obs.phi_4_R = motor_8009[1]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_R = motor_8009[1]->base.measure.speed_rad;
    obs.x_L     = motor_3508[0]->base.measure.total_angle*WHEEL_R;
    obs.dx_L    = motor_3508[0]->base.measure.speed_rad*WHEEL_R;
    obs.x_R     = motor_3508[1]->base.measure.total_angle*WHEEL_R;
    obs.dx_R    = motor_3508[1]->base.measure.speed_rad*WHEEL_R;

    /*
     * ==================== IMU 坐标系映射说明 ====================
     *
     * 1. 底层已知问题（未改）
     *    QuaternionEKF.c 中四元数反解欧拉角时 roll/pitch 公式互换了：
     *      QEKF_INS.Pitch = atan2(...)  // 实际用的是标准 Roll 公式
     *      QEKF_INS.Roll  = asin(...)   // 实际用的是标准 Pitch 公式
     *    导致 module_ins.c 输出：
     *      euler_rad[0] = QEKF_INS.Roll  → 实际物理含义 = 标准 Pitch (前后倾斜)
     *      euler_rad[1] = QEKF_INS.Pitch → 实际物理含义 = 标准 Roll  (左右倾斜)
     *    这是玺佬的代码，不在底层修改，在本层做映射修正。
     *
     * 2. 实车装配偏差
     *    C 板安装方向：X=左, Y=后, Z=上
     *    标准坐标系：  X=前, Y=左, Z=上
     *    两者相差绕 Z 轴 -90deg 旋转：标准X = -C板Y, 标准Y = C板X, 标准Z = C板Z
     *    BMI088 原始数据未做安装旋转补偿（IMU_Param 全零），此处手动完成坐标变换。
     *
     * 3. 最终映射结果
     *      obs.pitch  = euler_rad[0]  → 前后倾斜角 (标准 Pitch)
     *      obs.dpitch = -bmi->gyro[1] → 前后倾斜角速度 (C板Y=后, 取反=前)
     *      obs.roll   = euler_rad[1]  → 左右倾斜角 (标准 Roll)
     *      obs.droll  =  bmi->gyro[0] → 左右倾斜角速度 (C板X=左)
     *      obs.yaw    = euler_rad[2]  → 偏航角 (标准 Yaw, Z=上, 无需变换)
     * ============================================================
     */
    obs.pitch   = ins->euler_rad[0];
    obs.dpitch  = -bmi->gyro[1];
    obs.roll    = ins->euler_rad[1];
    obs.droll   = bmi->gyro[0];
    obs.yaw     = ins->euler_rad[2];
    obs.initialized = 1;
}

void observer_update(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[])
{
    obs.phi_1_L = motor_8009[3]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_L = motor_8009[3]->base.measure.speed_rad;
    obs.phi_4_L = motor_8009[0]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_L = motor_8009[0]->base.measure.speed_rad;
    obs.phi_1_R = motor_8009[2]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_R = motor_8009[2]->base.measure.speed_rad;
    obs.phi_4_R = motor_8009[1]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_R = motor_8009[1]->base.measure.speed_rad;
    obs.x_L     = motor_3508[0]->base.measure.total_angle*WHEEL_R;
    obs.dx_L    = motor_3508[0]->base.measure.speed_rad*WHEEL_R;
    obs.x_R     = motor_3508[1]->base.measure.total_angle*WHEEL_R;
    obs.dx_R    = motor_3508[1]->base.measure.speed_rad*WHEEL_R;

    obs.pitch   = ins->euler_rad[0];
    obs.dpitch  = -bmi->gyro[1];
    obs.roll    = ins->euler_rad[1];
    obs.droll   = bmi->gyro[0];
    obs.yaw     = ins->euler_rad[2];
}

const observer *observer_get(void)
{
    if (obs.initialized)
        return &obs;
    return NULL;
}
