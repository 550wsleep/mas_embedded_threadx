/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-24 17:01:09
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-07-25 11:20:07
 * @FilePath: \mas_embedded_threadx\apps\hero2\chassis_board\dm8009test.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "dm8009test.h"
#include "module_offline.h"
#include "motor_def.h"
#include "motor_damiao.h"
#include "user_lib.h"
#include <stdint.h>

static DM_Motor_t  *chassis_motor = NULL;

void chassis_init(void)  
{
    Motor_Init_Config_s chassis_motor_config = {
        .offline_init_config = {
            .name       = "dm8009",
            .beep_times = 1,
            .timeout_ms = 1000,
            .enable     = 1,
        },

        
        .setting_init_config = {
                .loop_type             =OPEN_LOOP ,
                .enableflag                = 1,

        },
        .motor_init_info = {
                .motor_type      = DM8009,
                .max_torque      = 30.0f,
        },
        
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config = {
            .can = {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 0x02,
                .rx_id = 0xf2,
            },
            },
    };

    chassis_motor = Motor_DM_Init(&chassis_motor_config, DM_MIT_MODE);
}

void chassis_func(void)
{
    if (chassis_motor != NULL)
    {
        if (!Module_Offline_get_device_status(chassis_motor->base.offline_dev))
        {
            Motor_DM_Start(chassis_motor);   
            Motor_DM_SetRef(chassis_motor, 1.0f);   
        } 
        else
        {
            Motor_DM_Stop(chassis_motor);
        }
    }
  
}

