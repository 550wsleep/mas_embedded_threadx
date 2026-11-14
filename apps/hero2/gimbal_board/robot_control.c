/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-11-12 23:22:34
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-11-13 22:22:30
 * @FilePath: \mas_embedded_threadx\apps\hero2\gimbal_board\robot_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_control.h"
#include "robot_func.h"
#include "hero2_def.h"
#include "module_boardcomm.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "gimbal_func.h"
#include "motor_base.h"
#include "power_control.h"
#include <math.h>

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

static GimbalToChassis_cmd_t     chassis_send_cmd; 
static ChassisToGimbal_referee_t chassis_upload_data;
static Gimbal_Ctrl_Cmd_t gimbal_cmd;

static void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        Module_BoardComm_Receive(&chassis_upload_data,sizeof(ChassisToGimbal_referee_t));

        RemoteControlSet(&gimbal_cmd);

        float   yaw_angle_rad = 0.0f;
        uint8_t yaw_valid     = gimbal_func(&gimbal_cmd, &yaw_angle_rad);

        /* 板件通讯 */
        chassis_send_cmd.vx           = gimbal_cmd.vx_cmd;
        chassis_send_cmd.offset_angle = yaw_valid ? (int16_t)lroundf(yaw_angle_rad * 1000.0f) : 0;
        Module_BoardComm_Send((uint8_t *)&chassis_send_cmd,sizeof(GimbalToChassis_cmd_t));

        Motor_ControlAll();
        PowerControl_Update();
        Motor_ApplyAll();

        tx_thread_sleep(2);
    }
}

int robot_control_init(void)
{
    if (gimbal_init() != 0) return -1;

    tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 30, 30,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
    return 0;
}
