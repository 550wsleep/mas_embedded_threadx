/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-11-12 23:22:34
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-11-12 23:46:43
 * @FilePath: \mas_embedded_threadx\apps\hero2\gimbal_board\robot_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_control.h"
#include "robot_func.h"
#include "hero2_def.h"
#include "module_boardcomm.h"  
#include "tx_api.h"           
#include "bsp_def.h"           

static TX_THREAD                  robot_control_thread;           /* ← 这俩变量没声明! */
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

static GimbalToChassis_cmd_t     chassis_send_cmd; 
static ChassisToGimbal_referee_t chassis_upload_data;
static Gimbal_Ctrl_Cmd_t chassis_cmd;

static void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        RemoteControlSet(&chassis_cmd);

        /* 板件通讯 */
        chassis_send_cmd.vx           = chassis_cmd.body_target_dx_gb;
        chassis_send_cmd.offset_angle = 0;

        Module_BoardComm_Send((uint8_t *)&chassis_send_cmd,sizeof(GimbalToChassis_cmd_t));

        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    Module_BoardComm_RegisterRxBuffer(&chassis_upload_data, sizeof(ChassisToGimbal_referee_t));

    tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 30, 30,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
}
