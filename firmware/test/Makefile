# ホストPC（gcc）で動く単体試験。Pico SDK は不要。
#   make -C firmware/test        ... ビルドして全部走らせる（最後に PASS / FAIL を表示）
# 対象は SDK に触らない純粋モジュール: attitude, controller(共通部分), controller_pid, mixer, thrust_model, params,
#   battery, cycle_timer（この2つはポート batt_adc_* / fc_clock_* を fake_port.c の偽物で置き換えて試験する）
# ③b: flight_state, guard, emergency_descent, control_loop, mailbox, ipc_flags（ポート imu / motor_pwm / 時計 / WDT / batt_adc は
#   fake_port.c の偽物。mailbox / ipc_flags は本物をそのままリンクし、試験が「PC」と「コア1」の役を演じる）
# 先に purity で src/*.c を全部ホストgccでコンパイルし、SDKのincludeが混ざったら失敗させる（docs/hw_boundary.md）。
CC      ?= gcc
SRC_DIR := ../src
INC_DIR := ../include
CFLAGS  := -std=c11 -O1 -g -Wall -Wextra -Wdouble-promotion -Wfloat-conversion -Werror -I$(INC_DIR)
SRCS    := test_main.c $(SRC_DIR)/attitude.c $(SRC_DIR)/controller.c $(SRC_DIR)/controller_pid.c \
           $(SRC_DIR)/mixer.c $(SRC_DIR)/imu_mount.c $(SRC_DIR)/thrust_model.c $(SRC_DIR)/params.c \
           $(SRC_DIR)/battery.c $(SRC_DIR)/cycle_timer.c \
           $(SRC_DIR)/flight_state.c $(SRC_DIR)/guard.c $(SRC_DIR)/emergency_descent.c $(SRC_DIR)/control_loop.c $(SRC_DIR)/mailbox.c $(SRC_DIR)/ipc_flags.c \
           test_fsm.c test_guard_descent.c test_ipc.c test_loop.c fake_port.c
PURE_SRCS := $(wildcard $(SRC_DIR)/*.c)
HDRS    := $(wildcard $(INC_DIR)/*.h)

.PHONY: all run clean purity
all: purity run

# src/*.c（port_pico は含まない）を -c だけでコンパイル。Pico SDK のヘッダが無いのでSDKに触れば失敗する
purity:
	@for f in $(PURE_SRCS); do \
	  $(CC) $(CFLAGS) -c $$f -o /dev/null || exit 1; \
	done
	@echo "purity OK: $(words $(PURE_SRCS)) files in src/ compile on host"

test_main: $(SRCS) $(HDRS) fake_port.h test_common.h
	$(CC) $(CFLAGS) $(SRCS) -o $@ -lm

run: test_main
	./test_main

clean:
	rm -f test_main
