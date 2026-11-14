# ソース一覧。SDK の呼び出しは一切書かない（ホストPCのCMake、たとえばシミュレータからも include() できる）。
#   FC_INCLUDE_DIR         ... ヘッダの場所
#   FC_CORE_SOURCES        ... 純粋C11。そのまま全部リンクする
#   FC_PORT_PICO_SOURCES   ... Pico SDK に触る実機用。シミュレータは使わず、同じヘッダを自前の.cで実装する
#   FC_CPP_MODULES         ... 入力（リスト、既定は空）。C++版に切り替えるモジュール名。
#                              使える名前: thrust_model mixer controller_pid attitude
#                              例: -DFC_CPP_MODULES="thrust_model;mixer"
#                              全部C++なら "thrust_model;mixer;controller_pid;attitude"
#                              （以前の案 FC_CONTROL_LANG=CPP 相当）。src_cpp/<m>.cpp が src/<m>.c の代わりになる
#   FC_CPP_COMPILE_OPTIONS ... 出力。.cpp に付けるコンパイルオプション（呼び出し側が $<COMPILE_LANGUAGE:CXX> で適用する）
# 境界の説明: docs/hw_boundary.md
get_filename_component(_FC_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(FC_INCLUDE_DIR ${_FC_ROOT}/include)

set(FC_CORE_SOURCES
    ${_FC_ROOT}/src/params.c
    ${_FC_ROOT}/src/attitude.c
    ${_FC_ROOT}/src/controller.c
    ${_FC_ROOT}/src/controller_pid.c
    ${_FC_ROOT}/src/mixer.c
    ${_FC_ROOT}/src/imu_mount.c
    ${_FC_ROOT}/src/thrust_model.c
    ${_FC_ROOT}/src/battery.c
    ${_FC_ROOT}/src/cycle_timer.c
    ${_FC_ROOT}/src/flight_state.c
    ${_FC_ROOT}/src/guard.c
    ${_FC_ROOT}/src/emergency_descent.c
    ${_FC_ROOT}/src/control_loop.c
    ${_FC_ROOT}/src/mailbox.c
    ${_FC_ROOT}/src/ipc_flags.c
)

# ---- C++版への切り替え（提案: kanato が1つずつ書き換えるので、モジュールごとに独立して切り替える）----
# CACHE にはしない: 呼び出し側（シミュレータ）が include 前に set() した普通の変数も、-D も、どちらも効くように
if(NOT DEFINED FC_CPP_MODULES)
    set(FC_CPP_MODULES "")
endif()
set(_FC_CPP_ALLOWED thrust_model mixer controller_pid attitude)

foreach(_m IN LISTS FC_CPP_MODULES)
    list(FIND _FC_CPP_ALLOWED ${_m} _ok)   # IN_LIST は古いポリシーで使えないのでFINDで書く
    if(_ok EQUAL -1)
        message(FATAL_ERROR "FC_CPP_MODULES: '${_m}' は使えない名前。使えるのは: ${_FC_CPP_ALLOWED}")
    endif()
    if(NOT EXISTS ${_FC_ROOT}/src_cpp/${_m}.cpp)
        message(FATAL_ERROR "FC_CPP_MODULES: ${_FC_ROOT}/src_cpp/${_m}.cpp が無い。先に書くか、FC_CPP_MODULES から ${_m} を外す")
    endif()
    list(FIND FC_CORE_SOURCES ${_FC_ROOT}/src/${_m}.c _i)
    if(_i EQUAL -1)
        message(FATAL_ERROR "FC_CPP_MODULES: FC_CORE_SOURCES に src/${_m}.c が無い（二重指定？）")
    endif()
    list(REMOVE_AT FC_CORE_SOURCES ${_i})
    list(INSERT FC_CORE_SOURCES ${_i} ${_FC_ROOT}/src_cpp/${_m}.cpp)
endforeach()

# .cpp に付けるフラグ（C側の -Wall -Wextra に、ホスト試験と同じ変換警告を足す。-Werror は付けない）
set(FC_CPP_COMPILE_OPTIONS -fno-exceptions -fno-rtti -Wall -Wextra -Wdouble-promotion -Wfloat-conversion)

set(FC_PORT_PICO_SOURCES
    ${_FC_ROOT}/src/port_pico/imu_lsm6dsv16x.c
    ${_FC_ROOT}/src/port_pico/motor_pwm.c
    ${_FC_ROOT}/src/port_pico/batt_adc_pico.c
    ${_FC_ROOT}/src/port_pico/clock_wdt_pico.c
)
