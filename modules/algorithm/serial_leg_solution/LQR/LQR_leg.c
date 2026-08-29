/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 12:10:31
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-26 19:24:01
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\LQR\LQR_leg.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "LQR_leg.h"
#include "solution_def.h"
#include "observer.h"
#include "kinematics.h"

static LQR LQR_ctrl;

float K_p0[4] = {20.4392, 94.2238, -155.5329, 1.7024};   // K[0]
float K_p1[4] = {55.9817, -50.4603, -8.3689, 0.2381};    // K[1]
float K_p2[4] = {-181.8475, 219.3275, -96.9871, 3.4664}; // K[2]
float K_p3[4] = {-103.5657, 138.6344, -72.4948, 1.9338}; // K[3]
float K_p4[4] = {275.7576, -157.1632, -21.7816, 31.6595};// K[4]
float K_p5[4] = {27.8745, -17.9068, -1.3970, 4.1551};    // K[5]
float K_p6[4] = {1497.2384, -1401.4599, 420.1510, 13.5622};// K[6]
float K_p7[4] = {111.2058, -131.8536, 59.7382, 0.6340};  // K[7]
float K_p8[4] = {431.5558, -236.0704, -41.0164, 48.6301};// K[8]
float K_p9[4] = {316.9914, -189.6058, -18.4850, 36.9037};// K[9]
float K_p10[4] = {1343.4311, -1608.5975, 710.8411, -33.7040};// K[10]
float K_p11[4] = {153.3062, -189.6469, 88.1234, -8.8669};// K[11]


void LQR_K_calc(float *K, float L_0)
{
    float L0_2 = L_0 * L_0;
    float L0_3 = L0_2 * L_0;
    
    K[0] = K_p0[0]*L0_3 + K_p0[1]*L0_2 + K_p0[2]*L_0 + K_p0[3];
    K[1] = K_p1[0]*L0_3 + K_p1[1]*L0_2 + K_p1[2]*L_0 + K_p1[3];
    K[2] = K_p2[0]*L0_3 + K_p2[1]*L0_2 + K_p2[2]*L_0 + K_p2[3];
    K[3] = K_p3[0]*L0_3 + K_p3[1]*L0_2 + K_p3[2]*L_0 + K_p3[3];
    K[4] = K_p4[0]*L0_3 + K_p4[1]*L0_2 + K_p4[2]*L_0 + K_p4[3];
    K[5] = K_p5[0]*L0_3 + K_p5[1]*L0_2 + K_p5[2]*L_0 + K_p5[3];
    K[6] = K_p6[0]*L0_3 + K_p6[1]*L0_2 + K_p6[2]*L_0 + K_p6[3];
    K[7] = K_p7[0]*L0_3 + K_p7[1]*L0_2 + K_p7[2]*L_0 + K_p7[3];
    K[8] = K_p8[0]*L0_3 + K_p8[1]*L0_2 + K_p8[2]*L_0 + K_p8[3];
    K[9] = K_p9[0]*L0_3 + K_p9[1]*L0_2 + K_p9[2]*L_0 + K_p9[3];
    K[10] =	K_p10[0]*L0_3 +	K_p10[1]*L0_2 +	K_p10[2]*L_0 +  K_p10[3];
   	K[11] =	K_p11[0]*L0_3 +	K_p11[1]*L0_2 +	K_p11[2]*L_0 +	K_p11[3];
}

void LQR_calc(float target_x, float target_dx)
{
    const fk *fk_left = kinematics_get_left();
    const fk *fk_right = kinematics_get_right();
    const observer *obs = observer_get();

    // 计算自适应K矩阵
    float K_l[12], K_r[12];
    LQR_K_calc(K_l, fk_left->L_0);
    LQR_K_calc(K_r, fk_right->L_0);
    
    // 获取状态量
    float theta_l = fk_left->theta;
    float dtheta_l = fk_left->dtheta;
    float theta_r = fk_right->theta;
    float dtheta_r = fk_right->dtheta;
    float x = (obs->x_L + obs->x_R)/2.0f;
    float dx = (obs->dx_L + obs->dx_R) / 2.0f;
    float Pitch = obs->pitch;
    float dPitch = obs->dpitch;

    // 左腿控制
    LQR_ctrl.TL = K_l[0]*theta_l + K_l[1]*dtheta_l + K_l[2]*(x-target_x) + K_l[3]*(dx-target_dx) + K_l[4]*Pitch + K_l[5]*dPitch;
    LQR_ctrl.Tpl = K_l[6]*theta_l + K_l[7]*dtheta_l + K_l[8]*(x-target_x) + K_l[9]*(dx-target_dx)+ K_l[10]*Pitch + K_l[11]*dPitch;
  
    // 右腿控制
    LQR_ctrl.TR = K_r[0]*theta_r + K_r[1]*dtheta_r + K_r[2]*(x-target_x) + K_r[3]*(dx-target_dx) + K_r[4]*Pitch + K_r[5]*dPitch;
    LQR_ctrl.Tpr = K_r[6]*theta_r + K_r[7]*dtheta_r + K_r[8]*(x-target_x) + K_r[9]*(dx-target_dx) + K_r[10]*Pitch + K_r[11]*dPitch;
}

const LQR *LQR_get(void)
{
    return &LQR_ctrl;
}

