#ifndef _GAS_SPRING_H_
#define _GAS_SPRING_H_

#include "solution_def.h"
#include "user_lib.h"

#define Fs              350.0f
#define curtine_theta2  12.26f/180.0f*PI
#define curtine_theta1  73.74f/180.0f*PI
#define L1              0.21f
#define L2              0.25f
#define L3              0.2021f
#define L6              0.0503f

#define GAS_SPRING_SIN_EPS  1e-4f
#define GAS_SPRING_L4_EPS   1e-4f

void gas_spring_init(void);
void gas_spring_calc_left(void);
void gas_spring_calc_right(void);
void gas_spring_calc(void);
const GasSpring *gas_spring_get(void);

#endif