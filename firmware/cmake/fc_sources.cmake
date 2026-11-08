# ソース一覧。SDK の呼び出しは一切書かない（ホストPCのCMake、たとえばシミュレータからも include() できる）。
#   FC_INCLUDE_DIR         ... ヘッダの場所
#   FC_CORE_SOURCES        ... 純粋C11。そのまま全部リンクする
#   FC_PORT_PICO_SOURCES   ... Pico SDK に触る実機用。シミュレータは使わず、同じヘッダを自前の.cで実装する
# 境界の説明: docs/hw_boundary.md
get_filename_component(_FC_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(FC_INCLUDE_DIR ${_FC_ROOT}/include)

set(FC_CORE_SOURCES
    ${_FC_ROOT}/src/params.c
    ${_FC_ROOT}/src/attitude.c
    ${_FC_ROOT}/src/controller.c
    ${_FC_ROOT}/src/controller_pid.c
    ${_FC_ROOT}/src/mixer.c
    ${_FC_ROOT}/src/thrust_model.c
    ${_FC_ROOT}/src/battery.c
    ${_FC_ROOT}/src/cycle_timer.c
)

set(FC_PORT_PICO_SOURCES
    ${_FC_ROOT}/src/port_pico/imu_lsm6dsv16x.c
    ${_FC_ROOT}/src/port_pico/motor_pwm.c
    ${_FC_ROOT}/src/port_pico/batt_adc_pico.c
    ${_FC_ROOT}/src/port_pico/clock_wdt_pico.c
)
