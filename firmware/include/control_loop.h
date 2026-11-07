/**
 * @file control_loop.h
 * @brief コア0の制御ループ本体。INT1待ち → FIFO → 姿勢 → 制御器 → ミキサー → PWM。
 *
 * 呼び出し順は docs/architecture.md の「1周期の流れ」を参照。
 * INT1の割り込みの中では何もせず（時刻記録のみ）、処理はメインループ側で行う。
 */
#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdint.h>
#include <stdbool.h>
#include "params.h"
#include "ipc.h"
#include "imu_lsm6dsv16x.h"
#include "attitude.h"
#include "controller.h"
#include "controller_pid.h"
#include "controller_policy.h"
#include "guard.h"
#include "emergency_descent.h"
#include "mixer.h"
#include "battery.h"
#include "flight_state.h"
#include "cycle_timer.h"

/** コア0の全状態。静的に1つだけ確保する（mallocなし）。 */
typedef struct {
    const params_t *params;
    cycle_timing_t timing;
    imu_int1_event_t last_int1;
    imu_batch_t batch;
    attitude_estimator_t att;
    battery_state_t batt;
    flight_fsm_t fsm;
    guard_t guard;
    descent_t descent;
    pid_controller_t pid;
    policy_controller_t policy;
    controller_id_t selected_controller; /**< 飛行指示待機で選んだ方 */
    controller_output_t prev_output;     /**< 前回出力（制御器の入力14の最後の4つ） */
    controller_output_t pid_shadow_out;
    control_packet_t target;             /**< 直近のメールボックス内容 */
    uint32_t last_mb_seq;
    uint64_t last_new_packet_us;         /**< 通信途絶の判定用 */
    bool prev_arm_flag;                  /**< 0→1検出用 */
    uint32_t imu_fail_count;
} control_loop_t;

/**
 * 全モジュールを初期化する（IMU・PWM・ADC・WDT・状態機械）。
 * 呼ぶコア: コア0。頻度: 起動時に1回。ipc_init() の後。IMU失敗や
 * WDT再起動なら状態を非常停止にして戻る（ループは動かし続ける）。
 */
void control_loop_init(control_loop_t *c, const params_t *params);

/**
 * 1周期ぶんの処理を行う（INT1が来た後の処理。待ちは含まない）。
 * 呼ぶコア: コア0。頻度: 約940Hz。ステップ③の試験ではこれを単体で呼べる。
 */
void control_loop_run_once(control_loop_t *c);

/**
 * メインループ。imu_wait_int1(5ms) → run_once を繰り返す。タイムアウト時は
 * 飛行中ならモーターを止めて非常停止へ。戻らない。
 * 呼ぶコア: コア0（main から）。頻度: 起動時に1回呼んだら戻らない。
 */
void control_loop_run(control_loop_t *c);

#endif /* CONTROL_LOOP_H */
