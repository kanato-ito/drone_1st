# 初代C ファームウェア（Pico 2 W / Pico SDK）
# ターゲット:
#   imu_fifo_test ... ステップ②のブレッドボード試験アプリ（今回）
#   drone_fw      ... 本体（ステップ③以降で追加。末尾の雛形を参照）
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# SDK の取り込み（project() より前に置く）
include(pico_sdk_import.cmake)

project(drone_fw C CXX ASM)

pico_sdk_init()

# ---- ソース一覧（SDKに触らない。ホストPCのCMakeからも使う）----
include(${CMAKE_CURRENT_LIST_DIR}/cmake/fc_sources.cmake)

# ---- ライブラリ fc_core: 純粋C11の本体（SDK不要。ホストPCでもそのままビルドできる）----
add_library(fc_core STATIC ${FC_CORE_SOURCES})
target_include_directories(fc_core PUBLIC ${FC_INCLUDE_DIR})
target_compile_options(fc_core PRIVATE -Wall -Wextra
    $<$<COMPILE_LANGUAGE:CXX>:${FC_CPP_COMPILE_OPTIONS}>)
if(FC_CPP_MODULES)
    message(STATUS "fc_core: C++版のモジュール = ${FC_CPP_MODULES}")
else()
    message(STATUS "fc_core: C++版のモジュール = (なし。全部C)")
endif()

# ---- ライブラリ fc_port_pico: Pico SDK に触る部分（差し替え境界。docs/hw_boundary.md）----
add_library(fc_port_pico STATIC ${FC_PORT_PICO_SOURCES})
target_link_libraries(fc_port_pico PUBLIC
    fc_core
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_sync
    hardware_pwm
    hardware_adc
    hardware_clocks
    hardware_watchdog
)
target_compile_options(fc_port_pico PRIVATE -Wall -Wextra)

# ---- 試験アプリ imu_fifo_test ----
add_executable(imu_fifo_test apps/imu_fifo_test.c)
target_compile_options(imu_fifo_test PRIVATE -Wall -Wextra)
target_link_libraries(imu_fifo_test PRIVATE fc_core fc_port_pico)

pico_set_program_name(imu_fifo_test "imu_fifo_test")
pico_set_program_version(imu_fifo_test "0.1")

# 出力は USB のみ（UART は使わない）
pico_enable_stdio_usb(imu_fifo_test 1)
pico_enable_stdio_uart(imu_fifo_test 0)

pico_add_extra_outputs(imu_fifo_test)

# ---- 試験アプリ control_bench（ステップ③a）----
# 安全のため、モーターへのPWM出力は既定で無効（デューティを計算して表示するだけ、ピンには0を出す）。
# 有効にするには -DFC_BENCH_PWM_ENABLE=ON。その場合は必ずプロペラを外すこと（docs/step3a_control_chain.md）。
option(FC_BENCH_PWM_ENABLE "control_bench でモーターPWMを実際に出す（プロペラを外すこと）" OFF)

add_executable(control_bench apps/control_bench.c)
target_compile_options(control_bench PRIVATE -Wall -Wextra)
target_link_libraries(control_bench PRIVATE fc_core fc_port_pico)
if(FC_BENCH_PWM_ENABLE)
    message(WARNING "FC_BENCH_PWM_ENABLE=ON: control_bench がモーターPWMを出します。プロペラを外してください")
    target_compile_definitions(control_bench PRIVATE FC_BENCH_PWM_ENABLE=1)
endif()

pico_set_program_name(control_bench "control_bench")
pico_set_program_version(control_bench "0.1")
pico_enable_stdio_usb(control_bench 1)
pico_enable_stdio_uart(control_bench 0)
pico_add_extra_outputs(control_bench)

# ---- 試験アプリ loop_bench（ステップ③b）----
# 状態機械つきの制御ループをブレッドボードで回す。PWM出力は control_bench と同じく既定で無効
# （上の FC_BENCH_PWM_ENABLE=ON のときだけ有効。必ずプロペラを外す。デューティ上限 0.30）。
# mailbox / ipc_flags は src/ の本物（fc_core）。ベンチではコア0がPC役でメールボックスに書く（書く人は1人）。
add_executable(loop_bench apps/loop_bench.c)
target_compile_options(loop_bench PRIVATE -Wall -Wextra)
target_link_libraries(loop_bench PRIVATE fc_core fc_port_pico)
if(FC_BENCH_PWM_ENABLE)
    target_compile_definitions(loop_bench PRIVATE FC_BENCH_PWM_ENABLE=1)
endif()

pico_set_program_name(loop_bench "loop_bench")
pico_set_program_version(loop_bench "0.1")
pico_enable_stdio_usb(loop_bench 1)
pico_enable_stdio_uart(loop_bench 0)
pico_add_extra_outputs(loop_bench)

# ---- 本体 drone_fw（ステップ③以降）の雛形 ----
# add_executable(drone_fw src/main.c ...)
# target_link_libraries(drone_fw PRIVATE fc_core fc_port_pico)
# pico_add_extra_outputs(drone_fw)
