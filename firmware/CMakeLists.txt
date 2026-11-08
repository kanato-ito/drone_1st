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

# ---- ライブラリ fc_imu: IMU と時計。試験アプリと本体の両方からリンクする ----
add_library(fc_imu STATIC
    src/imu_lsm6dsv16x.c
    src/cycle_timer.c
)
target_include_directories(fc_imu PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(fc_imu PUBLIC
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_sync
)
target_compile_options(fc_imu PRIVATE -Wall -Wextra)

# ---- 試験アプリ imu_fifo_test ----
add_executable(imu_fifo_test apps/imu_fifo_test.c)
target_compile_options(imu_fifo_test PRIVATE -Wall -Wextra)
target_link_libraries(imu_fifo_test PRIVATE fc_imu)

pico_set_program_name(imu_fifo_test "imu_fifo_test")
pico_set_program_version(imu_fifo_test "0.1")

# 出力は USB のみ（UART は使わない）
pico_enable_stdio_usb(imu_fifo_test 1)
pico_enable_stdio_uart(imu_fifo_test 0)

pico_add_extra_outputs(imu_fifo_test)

# ---- 本体 drone_fw（ステップ③以降）の雛形 ----
# add_executable(drone_fw src/main.c ...)
# target_link_libraries(drone_fw PRIVATE fc_imu)
# pico_add_extra_outputs(drone_fw)
