#include "car_state.h"
#include "kinematics.h"
#include <math.h>

static car_state_e car_state = STATE_STANDUP;
static uint16_t    stable_cnt = 0;

#define CAR_DONE_THETA  0.30f /* 两腿 |θ|<17° 判站立完成 */
#define CAR_DONE_CYCLES 1     /* 无防抖: 瞬时满足即切 */

void car_state_init(void)
{
    car_state = STATE_STANDUP;
    stable_cnt = 0;
}

car_state_e car_state_get(void)
{
    return car_state;
}

void car_state_update(void)
{
    const fk *l = kinematics_get_left();
    const fk *r = kinematics_get_right();
    if (l == NULL || r == NULL)
        return;

    if (car_state == STATE_STANDUP)
    {
        if (fabsf(l->theta) < CAR_DONE_THETA && fabsf(r->theta) < CAR_DONE_THETA)
        {
            if (++stable_cnt >= CAR_DONE_CYCLES)
            {
                car_state = STATE_BALANCE;
                stable_cnt = 0; /* 切换后清零, 防溢出兜底 */
            }
        }
        else
        {
            stable_cnt = 0; /* 任一腿超窗, 重新攒 */
        }
    }
    /* BALANCE: latch 不回退 (掉线用 car_state_reset); 后续 JUMP/STEP 在此扩展转移 */
}

void car_state_reset(void)
{
    car_state = STATE_STANDUP;
    stable_cnt = 0;
}
