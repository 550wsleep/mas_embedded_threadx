# 先加载默认模板，再覆盖差异

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关（底盘+云台测试）
set(MODULES_SINGLE   OFFLINE REMOTE BMI088 INS MOTOR)

# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)

# REMOTE 参数
set(REMOTE_UART             huart3)
set(REMOTE_SOURCE           1)      # 1=sbus
set(REMOTE_VT_SOURCE        0)      # 0=none

# MOTOR 参数
set(MOTOR_TASK_STACK_SIZE   1024)
set(MOTOR_TASK_PRIORITY     12)
set(MOTOR_OFFLINE_ENABLE    1)
