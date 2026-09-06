#include "gas_spring.h"
#include "arm_math.h"
#include "observer.h"
#include "kinematics.h"

static GasSpring gs;

void gas_spring_init(void)
{
    gs.Fs_L = 0.0f;
    gs.Fs_R = 0.0f;
}

static float _calc_Fs(float phi_2, float phi_1, float L_0)
{
    float curtine_phi2 = phi_2 + PI - phi_1 - curtine_theta2;

    float L4_sq = L3*L3 + L6*L6 - 2.0f*L3*L6*arm_cos_f32(curtine_phi2);
    if (L4_sq < 0.0f) L4_sq = 0.0f;
    float L4 = sqrtf(L4_sq);
    if (L4 < GAS_SPRING_L4_EPS) L4 = GAS_SPRING_L4_EPS;

    float sin_den = arm_sin_f32(curtine_phi2 + curtine_theta2);
    if (fabsf(sin_den) < GAS_SPRING_SIN_EPS)
        sin_den = (sin_den >= 0.0f) ? GAS_SPRING_SIN_EPS : -GAS_SPRING_SIN_EPS;

    return Fs*L3*L6*arm_sin_f32(curtine_phi2)*L_0/(L1*L2*sin_den*L4);
}

void gas_spring_calc_left(void)
{
    const observer *obs = observer_get();
    const fk       *ki  = kinematics_get_left();
    if (obs == NULL || ki == NULL) return;
    gs.Fs_L = _calc_Fs(ki->phi_2, obs->phi_1_L, ki->L_0);
}

void gas_spring_calc_right(void)
{
    const observer *obs = observer_get();
    const fk       *ki  = kinematics_get_right();
    if (obs == NULL || ki == NULL) return;
    gs.Fs_R = _calc_Fs(ki->phi_2, obs->phi_1_R, ki->L_0);
}

void gas_spring_calc(void)
{
    gas_spring_calc_left();
    gas_spring_calc_right();
}

const GasSpring *gas_spring_get(void)
{
    return &gs;
}