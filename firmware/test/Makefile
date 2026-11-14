# ホストPC（gcc）で動く単体試験。Pico SDK は不要。
#   make -C firmware/test        ... ビルドして全部走らせる（最後に PASS / FAIL を表示）
# 対象は SDK に触らない純粋モジュール: attitude, controller(共通部分), controller_pid, mixer, thrust_model, params,
#   battery, cycle_timer（この2つはポート batt_adc_* / fc_clock_* を fake_port.c の偽物で置き換えて試験する）
# ③b: flight_state, guard, emergency_descent, control_loop, mailbox, ipc_flags（ポート imu / motor_pwm / 時計 / WDT / batt_adc は
#   fake_port.c の偽物。mailbox / ipc_flags は本物をそのままリンクし、試験が「PC」と「コア1」の役を演じる）
# 先に purity で src/*.c を全部ホストgccでコンパイルし、SDKのincludeが混ざったら失敗させる（docs/hw_boundary.md）。
#
# C++版への切り替え（src_cpp/README.md）:
#   make -C firmware/test CPP_MODULES="thrust_model mixer"
#     ... 指定モジュールだけ ../src/<m>.c の代わりに $(CPP_DIR)/<m>.cpp を g++ でコンパイルし、g++ でリンクする。
#     使える名前: thrust_model mixer controller_pid attitude。既定（空）は今まで通り全部C。
#   CPP_DIR=... で .cpp の置き場所を差し替えられる（仕組みの確認用。既定 ../src_cpp）。
# cxxheaders: include/*.h をC++から include できることを確認する（C専用ヘッダは除く）。all で purity と一緒に走る。
CC      ?= gcc
CXX     ?= g++
SRC_DIR := ../src
CPP_DIR ?= ../src_cpp
CPP_MODULES ?=
CPP_ALLOWED := thrust_model mixer controller_pid attitude
INC_DIR := ../include
CFLAGS  := -std=c11 -O1 -g -Wall -Wextra -Wdouble-promotion -Wfloat-conversion -Werror -I$(INC_DIR)
CXXFLAGS := -std=c++17 -O1 -g -Wall -Wextra -Wdouble-promotion -Wfloat-conversion -Werror -fno-exceptions -fno-rtti -I$(INC_DIR)
# <stdatomic.h> を使うC専用ヘッダ（ipc.h / control_loop.h は ipc_flags.h・spsc_ring.h 経由でC専用）
CXX_SKIP_HDRS := ipc_flags.h spsc_ring.h ipc.h control_loop.h
SRCS    := test_main.c $(SRC_DIR)/attitude.c $(SRC_DIR)/controller.c $(SRC_DIR)/controller_pid.c \
           $(SRC_DIR)/mixer.c $(SRC_DIR)/imu_mount.c $(SRC_DIR)/thrust_model.c $(SRC_DIR)/params.c \
           $(SRC_DIR)/battery.c $(SRC_DIR)/cycle_timer.c \
           $(SRC_DIR)/flight_state.c $(SRC_DIR)/guard.c $(SRC_DIR)/emergency_descent.c $(SRC_DIR)/control_loop.c $(SRC_DIR)/mailbox.c $(SRC_DIR)/ipc_flags.c \
           test_fsm.c test_guard_descent.c test_ipc.c test_loop.c fake_port.c
PURE_SRCS := $(wildcard $(SRC_DIR)/*.c)
HDRS    := $(wildcard $(INC_DIR)/*.h)

# ---- CPP_MODULES の検証と、ソース一覧の置き換え ----
$(foreach m,$(CPP_MODULES),\
  $(if $(filter $(m),$(CPP_ALLOWED)),,$(error CPP_MODULES: '$(m)' は使えない名前。使えるのは: $(CPP_ALLOWED))) \
  $(if $(wildcard $(CPP_DIR)/$(m).cpp),,$(error CPP_MODULES: $(CPP_DIR)/$(m).cpp が無い。先に書くか、CPP_MODULES から $(m) を外す)))
CPP_SRCS := $(foreach m,$(CPP_MODULES),$(CPP_DIR)/$(m).cpp)
C_SRCS   := $(filter-out $(foreach m,$(CPP_MODULES),$(SRC_DIR)/$(m).c),$(SRCS))
OBJ_DIR  := obj_cpp
C_OBJS   := $(patsubst %.c,$(OBJ_DIR)/c_%.o,$(notdir $(C_SRCS)))
CPP_OBJS := $(patsubst %.cpp,$(OBJ_DIR)/cpp_%.o,$(notdir $(CPP_SRCS)))
CXX_HDRS := $(filter-out $(CXX_SKIP_HDRS),$(notdir $(HDRS)))

.PHONY: all run clean purity cxxheaders
all: purity cxxheaders run

# src/*.c（port_pico は含まない）を -c だけでコンパイル。Pico SDK のヘッダが無いのでSDKに触れば失敗する
purity:
	@for f in $(PURE_SRCS); do \
	  $(CC) $(CFLAGS) -c $$f -o /dev/null || exit 1; \
	done
	@echo "purity OK: $(words $(PURE_SRCS)) files in src/ compile on host"

# include/*.h を1つずつ g++ で include するだけの翻訳単位を作って -c（C専用ヘッダは除く）
cxxheaders:
	@for h in $(CXX_HDRS); do \
	  printf '#include "%s"\n#include "%s"\n' $$h $$h | $(CXX) $(CXXFLAGS) -x c++ -c - -o /dev/null || { echo "cxxheaders NG: $$h"; exit 1; }; \
	done
	@echo "cxxheaders OK: $(words $(CXX_HDRS)) headers compile as C++ (skipped C-only: $(CXX_SKIP_HDRS))"

# C++ を混ぜるときは、C は $(CC)、C++ は $(CXX) で別々に .o にして、$(CXX) でリンクする。
# CPP_MODULES が空なら今まで通り1コマンド。（.cpp 側の変更も拾うため、毎回作り直す）
test_main: $(SRCS) $(CPP_SRCS) $(HDRS) fake_port.h test_common.h
ifeq ($(strip $(CPP_MODULES)),)
	$(CC) $(CFLAGS) $(SRCS) -o $@ -lm
else
	@echo "C++版: $(CPP_MODULES)"
	@mkdir -p $(OBJ_DIR)
	@for f in $(C_SRCS); do $(CC) $(CFLAGS) -c $$f -o $(OBJ_DIR)/c_$$(basename $$f .c).o || exit 1; done
	@for f in $(CPP_SRCS); do $(CXX) $(CXXFLAGS) -c $$f -o $(OBJ_DIR)/cpp_$$(basename $$f .cpp).o || exit 1; done
	$(CXX) $(C_OBJS) $(CPP_OBJS) -o $@ -lm
endif

run: test_main
	./test_main

clean:
	rm -f test_main $(OBJ_DIR)/*.o
