/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-25 19:45:44
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-07-25 20:34:19
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\solution_def.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-25 19:45:44
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-07-25 20:12:42
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\solution_def.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _SOLUTION_DEF_H_
#define _SOLUTION_DEF_H_

#include <stdint.h>

#define WHEEL_R 0.05f
#define phi_1_offset 0.0f
#define phi_4_offset 0.0f
//观测器
typedef struct
{
    float phi_1_L;
    float phi_4_L;
    float phi_1_R;
    float phi_4_R;
    float x_L;
    float dx_L;
    float x_R;
    float dx_R;
    float pitch;
    float dpitch;
    float roll;
    float yaw;
    uint8_t initialized ;
} observer;

//运动学解算
typedef struct
{
    float L_0;
    float theta;
    float dtheta;
} fk;

//LQR解算
typedef struct
{
    float T;
    float Tp;
} LQR;

//VMC解算
typedef struct
{
   float T_1;   
   float T_2;
} VMC;    
 
