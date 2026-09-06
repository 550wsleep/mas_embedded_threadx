#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "solution_def.h"

#define L_1       0.21f  
#define L_2       0.25f
#define L_3       0.25f
#define L_4       0.21f


void kinematics_calc(void);
void kinematics_init(void);
const fk *kinematics_get_left(void);
const fk *kinematics_get_right(void);

#endif