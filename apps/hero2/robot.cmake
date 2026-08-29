include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关（按板型覆盖默认值）
set(MODULES_CHASSIS   OFFLINE REMOTE MOTOR SERIAL_LEG_SOLUTION BMI088 INS  )

# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 开启蜂鸣器