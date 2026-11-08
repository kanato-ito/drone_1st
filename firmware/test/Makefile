# ホストPC（gcc）で動く単体試験。Pico SDK は不要。
#   make -C firmware/test        ... ビルドして全部走らせる（最後に PASS / FAIL を表示）
# 対象は SDK に触らない純粋モジュール: attitude, controller(共通部分), controller_pid, mixer, thrust_model, params
CC      ?= gcc
SRC_DIR := ../src
INC_DIR := ../include
CFLAGS  := -std=c11 -O1 -g -Wall -Wextra -Wdouble-promotion -Wfloat-conversion -Werror -I$(INC_DIR)
SRCS    := test_main.c $(SRC_DIR)/attitude.c $(SRC_DIR)/controller.c $(SRC_DIR)/controller_pid.c \
           $(SRC_DIR)/mixer.c $(SRC_DIR)/thrust_model.c $(SRC_DIR)/params.c
HDRS    := $(wildcard $(INC_DIR)/*.h)

.PHONY: all run clean
all: run

test_main: $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) $(SRCS) -o $@ -lm

run: test_main
	./test_main

clean:
	rm -f test_main
