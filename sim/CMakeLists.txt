# シミュレータ用に、ファームの純粋Cの部分とポートの偽物を共有ライブラリ（libfcsim）にまとめる。
# Pico SDK は使わない。ホストのコンパイラ（Linux: gcc、Mac: clang）でビルドする。
#
#   cmake -S sim -B sim/build -DCMAKE_BUILD_TYPE=Release
#   cmake --build sim/build
#
# できたライブラリ（sim/build/libfcsim.so、Macでは .dylib）を Python が ctypes で読む（sim/fcsim/fc.py）。
cmake_minimum_required(VERSION 3.16)
project(fcsim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(${CMAKE_CURRENT_LIST_DIR}/../firmware/cmake/fc_sources.cmake)   # FC_INCLUDE_DIR, FC_CORE_SOURCES

# ファーム本体はそのまま（FC_PORT_PICO_SOURCES は使わず、同じヘッダを c/port_sim.c で実装する）
add_library(fcsim SHARED
    ${FC_CORE_SOURCES}
    c/port_sim.c
    c/fcsim.c
)
target_include_directories(fcsim PRIVATE ${FC_INCLUDE_DIR} c)
target_link_libraries(fcsim PRIVATE m)

# 外から見える関数は fcsim.h の FCSIM_API だけにする
set_target_properties(fcsim PROPERTIES C_VISIBILITY_PRESET hidden)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # ファームの単体試験（firmware/test/Makefile）と同じ警告。ファーム側で警告が出たら直すきっかけにする
    target_compile_options(fcsim PRIVATE -Wall -Wextra -Wdouble-promotion -Wfloat-conversion)
endif()

# 出力先を sim/build 直下にそろえる（fc.py が探す場所）
set_target_properties(fcsim PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
