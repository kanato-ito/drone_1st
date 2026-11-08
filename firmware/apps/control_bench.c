/**
 * @file control_bench.c
 * @brief ステップ③aのブレッドボード試験: IMU → 姿勢推定 → 2段PID → ミキサー → 推力換算 → デューティ。
 *
 * ★安全★ モーターへのPWMは既定で「出さない」。デューティを計算して表示するだけで、ピンには0のまま。
 *   出すには CMake に -DFC_BENCH_PWM_ENABLE=ON が必要。その場合は必ずプロペラを外すこと。
 *   （出す場合も、デューティは FC_BENCH_DUTY_MAX で頭打ち、傾き60°超か INT1 途絶で全部0にして止める）
 *
 * 簡易手順（状態機械 flight_state はステップ③bで作るので、ここでは使わない）:
 *   1. 起動後、ボードを静止させる。姿勢推定が「静止が1秒続いた」と判定するとバイアスが求まる（WAIT 表示）
 *   2. ほぼ水平（アーム許容10°以内）なら、バイアスを固定して制御を回し始める（RUN 表示）
 *   3. 目標は水平・ヨーは開始時の向き、スロットルは固定の仮値 FC_BENCH_THRUST_G [g/モーター]
 * 画面は1秒に1行（printf は遅いのでループの時間を乱さない）。ループ処理時間は
 * 「INT1の立ち上がり〜デューティ計算（と有効時はPWM更新）完了」を測り、表示（printf）は含まない。
 *
 * ---- 合格条件（docs/step3a_control_chain.md に詳細）----
 *  1. 静止で roll/pitch ≈ 0（±1°）、bias[dps] が数dps以内で安定
 *  2. 手で傾けると roll/pitch が追従して符号が合う。戻すと0へ戻る
 *  3. 傾けると PID 出力と duty が「戻す向き」に動く（符号表は仮なので向きは台上確認のみ）
 *  4. proc_us の最大が 1064µs に余裕をもって収まる（目安: 最大 800µs 以下。平均はその半分以下）
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"

#include "board_pins.h"
#include "fc_types.h"
#include "cycle_timer.h"
#include "imu_lsm6dsv16x.h"
#include "params.h"
#include "attitude.h"
#include "controller.h"
#include "controller_pid.h"
#include "mixer.h"
#include "thrust_model.h"
#include "battery.h"
#include "motor_pwm.h"

#define REPORT_PERIOD_US 1000000u
#define RAD_TO_DEG       57.29577951308232f
#define LOOP_BUDGET_US   1064u      /* 1周期（IMUの実周期） */
#define BATT_EVERY_N     8u         /* 電池ADCは数周期に1回（電圧は遅い） */

/* 仮の固定スロットル [g/モーター]。ホバリングに必要な量は未測定（プロペラ無しの台上確認用の値） */
#ifndef FC_BENCH_THRUST_G
#define FC_BENCH_THRUST_G 10.0f
#endif
/* ADC未接続（USB給電だけ）のとき換算に使う仮の電圧 [V] と、「未接続」とみなす境目 */
#define BENCH_FAKE_BATT_V   3.8f
#define BENCH_NO_BATT_BELOW 2.0f
/* PWMを出す場合のデューティ上限（提案）。プロペラを外しても回るので念のため */
#ifndef FC_BENCH_DUTY_MAX
#define FC_BENCH_DUTY_MAX 0.30f
#endif

#ifdef FC_BENCH_PWM_ENABLE
#define PWM_ENABLED 1
#else
#define PWM_ENABLED 0
#endif

typedef struct {
    uint32_t cycles;
    uint64_t proc_sum_us;
    uint32_t proc_max_us;
    uint32_t over_budget;   /* 累計: 1064µs を超えた回数 */
} stats_t;

int main(void)
{
    stdio_init_all();
    /* 最初に必ずPWMを0にする（有効ビルドでもここでは0） */
    motor_pwm_init();
    motor_pwm_stop();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);

    printf("\n=== control_bench (step 3a: IMU -> attitude -> PID -> mixer -> duty) ===\n");
    printf("PWM出力: %s\n", PWM_ENABLED ? "有効 (!!) プロペラを外していることを確認。デューティ上限あり" : "無効（計算して表示するだけ。ピンは0）");

    params_t *prm = params_get_mutable();
    params_set_defaults(prm);
    if (!params_validate(prm)) {
        for (;;) { sleep_ms(2000); printf("params_validate 失敗。既定値を確認\n"); }
    }

    imu_config_t cfg = imu_config_default();
    imu_status_t st = imu_init(&cfg);
    printf("imu_init: %d (0=OK)  SPI実クロック %u Hz\n", (int)st, (unsigned)imu_spi_actual_baud_hz());
    if (st != IMU_OK) {
        for (;;) { sleep_ms(2000); printf("imu_init 失敗 (%d)。配線を確認して電源を入れ直す\n", (int)st); }
    }

    static imu_batch_t batch;                 /* 大きいので static */
    static attitude_estimator_t att;
    static pid_controller_t pid;
    static battery_state_t batt;
    static mixer_result_t mix;
    attitude_init(&att, &prm->attitude);
    controller_pid_init(&pid, &prm->pid);
    controller_t ctrl = controller_pid_handle(&pid);
    battery_init(&batt);

    const float dt_sample = imu_sample_period_s();
    controller_output_t prev_out = { 0.0f, { 0.0f, 0.0f, 0.0f } };
    controller_output_t out = prev_out;
    euler_t target = { 0.0f, 0.0f, 0.0f };    /* 目標: 水平。ヨーは開始時の向き（RUN開始時に入れる） */
    float duty[FC_NUM_MOTORS] = { 0, 0, 0, 0 };

    bool running = false;
    bool tripped = false;       /* 傾き過大の停止（PWM有効ビルドでは電源入れ直しまで保持） */
    bool no_batt = false;
    float batt_v_used = BENCH_FAKE_BATT_V;
    uint32_t cycle = 0, timeouts = 0, read_errors = 0;
    stats_t s = {0, 0, 0, 0};

    imu_int1_event_t ev = {0, 0};
    uint32_t last_report_us = fc_clock_now_us32();
    uint32_t sec = 0;

    for (;;) {
        bool got = imu_wait_int1(FC_INT1_TIMEOUT_US, &ev);
        if (!got) {
            timeouts++;
            /* INT1が来ない = 姿勢が更新できない。モーターを止め、電源入れ直しまで再開しない */
            motor_pwm_stop();
            if (running) tripped = true;
        } else {
            imu_status_t rs = imu_read_fifo(&batch);
            if (rs != IMU_OK) {
                read_errors++;
                motor_pwm_stop();
                if (running) tripped = true;
            } else {
                attitude_update_batch(&att, &batch, dt_sample);
                const attitude_state_t *as = attitude_get(&att);

                if ((cycle % BATT_EVERY_N) == 0) {
                    battery_update(&batt, &prm->battery, fc_clock_now_us32() / 1000u);
                    no_batt = batt.raw_v < BENCH_NO_BATT_BELOW;
                    batt_v_used = no_batt ? BENCH_FAKE_BATT_V : batt.voltage_v;
                }

                /* --- 待機 → 制御開始（簡易手順）--- */
                if (!running && attitude_is_still(&att) && attitude_bias_ready(&att) &&
                    attitude_tilt_rad(as) <= prm->safety.arm_level_tol_rad) {
                    attitude_set_bias_locked(&att, true);   /* アーム時にバイアス固定（rev22） */
                    ctrl.ops->reset(ctrl.self);             /* アーム時に積分0（rev22） */
                    target.yaw_rad = as->angle.yaw_rad;
                    prev_out = (controller_output_t){ 0.0f, { 0.0f, 0.0f, 0.0f } };
                    running = true;
                }

                if (running) {
                    controller_input_t in;
                    controller_input_make(as, &target, FC_BENCH_THRUST_G, &prev_out, &in);
                    if (!ctrl.ops->step(ctrl.self, &in, dt_sample * (float)batch.count, &out)) {
                        out = (controller_output_t){ 0.0f, { 0.0f, 0.0f, 0.0f } };
                    }
                    /* 前回出力は「実際に指令した値」。ミキサーで丸めた後の値ではなく制御器の出力(上限処理済み)を使う */
                    prev_out = out;

                    float vmax = thrust_model_max_g(&prm->airframe, batt_v_used);
                    mixer_mix(&prm->airframe, &out, vmax, &mix);
                    for (int i = 0; i < FC_NUM_MOTORS; i++) {
                        float d = thrust_model_to_duty(&prm->airframe, mix.motor_g[i], batt_v_used);
                        duty[i] = d;
                    }

#if PWM_ENABLED
                    if (attitude_tilt_rad(as) > prm->safety.tilt_estop_rad) tripped = true;   /* 保持 */
                    if (tripped) {
                        for (int i = 0; i < FC_NUM_MOTORS; i++) duty[i] = 0.0f;
                        motor_pwm_stop();
                    } else {
                        float lim[FC_NUM_MOTORS];
                        for (int i = 0; i < FC_NUM_MOTORS; i++) lim[i] = duty[i] > FC_BENCH_DUTY_MAX ? FC_BENCH_DUTY_MAX : duty[i];
                        motor_pwm_set(lim);
                    }
#else
                    /* PWM無効: ピンには何も出さない（起動時に0にしてある）。傾き過大は表示だけ */
                    tripped = attitude_tilt_rad(as) > prm->safety.tilt_estop_rad;
#endif
                }

                /* ここまでが「INT1から出力更新まで」。表示は含めない */
                uint32_t proc = (uint32_t)(fc_clock_now_us32() - ev.timestamp_us);
                s.cycles++;
                s.proc_sum_us += proc;
                if (proc > s.proc_max_us) s.proc_max_us = proc;
                if (proc > LOOP_BUDGET_US) s.over_budget++;
                cycle++;
            }
        }

        uint32_t now = fc_clock_now_us32();
        if ((uint32_t)(now - last_report_us) >= REPORT_PERIOD_US) {
            sec++;
            const attitude_state_t *as = attitude_get(&att);
            uint32_t avg = s.cycles ? (uint32_t)(s.proc_sum_us / s.cycles) : 0;
            printf("[%4us] %s att[deg] r=%+6.2f p=%+6.2f y=%+7.2f bias[dps]=(%+5.2f %+5.2f %+5.2f)",
                   (unsigned)sec, running ? (tripped ? "TRIP" : "RUN ") : "WAIT",
                   (double)(as->angle.roll_rad * RAD_TO_DEG), (double)(as->angle.pitch_rad * RAD_TO_DEG),
                   (double)(as->angle.yaw_rad * RAD_TO_DEG),
                   (double)(att.bias_rps.x * RAD_TO_DEG), (double)(att.bias_rps.y * RAD_TO_DEG),
                   (double)(att.bias_rps.z * RAD_TO_DEG));
            if (!running) {
                printf(" still=%d hold=%.1fs tilt=%.1fdeg", attitude_is_still(&att) ? 1 : 0,
                       (double)att.still_time_s, (double)(attitude_tilt_rad(as) * RAD_TO_DEG));
            } else {
                printf(" pid[g] T=%.1f r=%+5.2f p=%+5.2f y=%+5.2f duty=(%.3f %.3f %.3f %.3f) sat=%d",
                       (double)out.thrust_g, (double)out.torque_g[0], (double)out.torque_g[1], (double)out.torque_g[2],
                       (double)duty[0], (double)duty[1], (double)duty[2], (double)duty[3], mix.saturated ? 1 : 0);
            }
            printf(" Vbat=%.2f%s proc_us avg/max=%u/%u over=%u TMO=%u ERR=%u PWM=%s\n",
                   (double)batt_v_used, no_batt ? "(仮:未接続)" : "",
                   (unsigned)avg, (unsigned)s.proc_max_us, (unsigned)s.over_budget,
                   (unsigned)timeouts, (unsigned)read_errors, PWM_ENABLED ? "ON" : "OFF");
            /* 平均/最大は1秒ごとにクリア。over は累計 */
            s.cycles = 0; s.proc_sum_us = 0; s.proc_max_us = 0;
            last_report_us = now;
        }
    }
}
