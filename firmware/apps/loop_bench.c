/**
 * @file loop_bench.c
 * @brief ステップ③bのブレッドボード試験: 状態機械つきの制御ループ（control_loop）を実機で回す。
 *
 * コア0で control_loop_step を回し、周期の合間に USB シリアルから1文字コマンドを読んで「PC」の役を演じる
 * （mailbox にパケットを書き、旗を立てる）。mailbox / ipc_flags は src/ の本物。
 * 本来 mailbox を書くのはコア1だが、このベンチはコア0だけで動き、書く人が1人（コア0）なので
 * 周期の合間にコア0から書いても seqlock の約束は守られる。
 *
 * ★安全★ モーターへのPWMは既定で「出さない」（control_bench と同じ）。状態機械は動くが、飛行中でも
 *   ピンには0のまま。出すには CMake に -DFC_BENCH_PWM_ENABLE=ON が必要。その場合は必ずプロペラを外すこと。
 *   出す場合もデューティは FC_BENCH_DUTY_MAX（0.30）で頭打ち。
 *   電池をつないでいない（ADC 2V未満）ときは電池なしモードにして、電池低下で降下に入らないようにする。
 *
 * コマンド（ターミナルから1文字。Enter不要の端末を推奨）:
 *   c   PCと接続（パケットを50Hzで送り始める）      s   パケット送信を止める（通信途絶の試験）
 *   a   アーム旗の押す/離すを切り替える             d   ディスアーム（旗カウンタ + パケット旗を一瞬）
 *   e   非常停止（旗 + パケット旗。電源入れ直しまで戻らない）
 *   +/- スロットルを 0.05 ずつ増減（0..1）           0   スロットルを0に
 *   t   「PIDに取り上げ」旗の切り替え（方策が無いので表示が変わるだけ）
 *   w   WDT試験: 餌やりを止めて固まる（停止状態のときだけ受け付ける）。約100ms後に再起動 → 非常停止(wdt)
 *   h   ヘルプ
 * 画面は1秒に1行。printf は周期の処理（タイミング計測の区間）の外で行う。
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"

#include "board_pins.h"
#include "control_loop.h"
#include "motor_pwm.h"
#include "params.h"

#define REPORT_PERIOD_US   1000000u
#define PACKET_PERIOD_US   20000u      /* 50Hz */
#define DISARM_FLAG_US     300000u     /* パケットのディスアーム旗を立てておく時間 */
#define KEY_POLL_EVERY_N   8u
#define RAD_TO_DEG         57.29577951308232f
#define NO_BATT_BELOW_V    2.0f        /* これ未満なら電池なしとみなす */

#ifndef FC_BENCH_DUTY_MAX
#define FC_BENCH_DUTY_MAX 0.30f
#endif

#ifdef FC_BENCH_PWM_ENABLE
#define PWM_ENABLED 1
#else
#define PWM_ENABLED 0
#endif

/* 「PC」の役 */
typedef struct {
    bool     connected;
    bool     arm;
    bool     estop;
    bool     takeover;
    float    throttle;
    uint32_t disarm_until_us;
    uint32_t seq;
    uint32_t next_us;
    uint32_t sent;
    bool     disarm_flag_active;
} pc_t;

static control_loop_t L;     /* 大きいので static */
static pc_t pc;

static void pc_send(uint32_t now_us)
{
    control_packet_t p = { 0.0f, 0.0f, 0.0f, pc.throttle, 0u, ++pc.seq };
    if (pc.arm) p.flags |= CTRL_FLAG_ARM_REQ;
    if (pc.estop) p.flags |= CTRL_FLAG_ESTOP;
    if (pc.takeover) p.flags |= CTRL_FLAG_TAKEOVER_PID;
    if (pc.disarm_flag_active) p.flags |= CTRL_FLAG_DISARM;
    mailbox_write(&p);
    pc.sent++;
    pc.next_us = now_us + PACKET_PERIOD_US;
}

static void print_help(void)
{
    printf("コマンド: c=接続 s=送信停止 a=アーム押す/離す d=ディスアーム e=非常停止 +/-=スロットル 0=スロットル0 "
           "t=取り上げ旗 w=WDT試験 h=ヘルプ\n");
}

static void handle_key(int ch, uint32_t now_us)
{
    switch (ch) {
    case 'c': pc.connected = true; pc.next_us = now_us; printf(">> 接続（パケット送信開始）\n"); break;
    case 's': pc.connected = false; printf(">> 送信停止（通信途絶の試験）\n"); break;
    case 'a': pc.arm = !pc.arm; printf(">> アーム旗 %s\n", pc.arm ? "押した(1)" : "離した(0)"); pc.next_us = now_us; break;
    case 'd':
        ipc_flags_request_disarm(ipc_flags());
        pc.arm = false;
        pc.disarm_flag_active = true; pc.disarm_until_us = now_us + DISARM_FLAG_US;
        printf(">> ディスアーム（旗カウンタ+1、パケット旗を一瞬）\n"); pc.next_us = now_us; break;
    case 'e':
        ipc_flags_request_estop(ipc_flags());
        pc.estop = true;
        printf(">> 非常停止の要求（電源入れ直しまで戻らない）\n"); pc.next_us = now_us; break;
    case '+': pc.throttle += 0.05f; if (pc.throttle > 1.0f) pc.throttle = 1.0f; pc.next_us = now_us; break;
    case '-': pc.throttle -= 0.05f; if (pc.throttle < 0.0f) pc.throttle = 0.0f; pc.next_us = now_us; break;
    case '0': pc.throttle = 0.0f; pc.next_us = now_us; break;
    case 't': pc.takeover = !pc.takeover; printf(">> 取り上げ旗 %d\n", pc.takeover ? 1 : 0); pc.next_us = now_us; break;
    case 'w':
        if (flight_fsm_motors_allowed(L.fsm.state)) {
            printf(">> WDT試験は停止状態のときだけ（今は飛行中/降下中）\n");
        } else {
            printf(">> WDT試験: 餌やりを止めます。約%ums後に再起動し、非常停止(wdt_reboot)で立ち上がるはず\n",
                   (unsigned)L.params->safety.wdt_timeout_ms);
            for (;;) { tight_loop_contents(); }
        }
        break;
    case 'h': case '?': print_help(); break;
    default: break;
    }
}

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

    printf("\n=== loop_bench (step 3b: 状態機械つき制御ループ) ===\n");
    printf("PWM出力: %s\n", PWM_ENABLED ? "有効 (!!) プロペラを外していることを確認。デューティ上限あり" : "無効（計算して表示するだけ。ピンは0）");

    params_t *prm = params_get_mutable();
    params_set_defaults(prm);
    if (!params_validate(prm)) {
        for (;;) { sleep_ms(2000); printf("params_validate 失敗。既定値を確認\n"); }
    }

    /* ここでWDTが始まる（最後）。以降の printf は短い行だけにし、固まらせない */
    control_loop_init(&L, prm);
#if PWM_ENABLED
    L.motor_output_enabled = true;
    L.motor_duty_max = FC_BENCH_DUTY_MAX;
#else
    L.motor_output_enabled = false;
#endif
    if (L.batt.raw_v < NO_BATT_BELOW_V) L.bench_no_battery = true;

    printf("init: state=%s estop=%s imu_ok=%d wdt_reboot=%d WDT=%ums 電池%s\n",
           flight_state_name(L.fsm.state), estop_reason_name(L.fsm.estop_reason),
           L.boot_imu_ok ? 1 : 0, L.wdt_reboot ? 1 : 0, (unsigned)prm->safety.wdt_timeout_ms,
           L.bench_no_battery ? "なし(3.8V固定・低下判定なし)" : "あり");
    print_help();

    uint32_t last_report_us = fc_clock_now_us32();
    uint32_t sec = 0, cyc = 0;
    uint64_t proc_sum = 0;
    uint32_t proc_max = 0, proc_n = 0;

    for (;;) {
        control_loop_step(&L);   /* ここで1周期（INT1待ち + 処理）。タイミング計測は中で完結 */

        /* ---- 以降は周期の外: 統計、「PC」の役、キー入力、表示 ---- */
        uint32_t proc = L.timing.proc_us;
        proc_sum += proc; proc_n++;
        if (proc > proc_max) proc_max = proc;
        cyc++;

        uint32_t now = fc_clock_now_us32();
        if (pc.disarm_flag_active && (int32_t)(now - pc.disarm_until_us) >= 0) {
            pc.disarm_flag_active = false; pc.next_us = now;
        }
        if ((cyc % KEY_POLL_EVERY_N) == 0) {
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) handle_key(ch, now);
        }
        if (pc.connected && (int32_t)(now - pc.next_us) >= 0) pc_send(now);

        if ((uint32_t)(now - last_report_us) >= REPORT_PERIOD_US) {
            sec++;
            const attitude_state_t *as = attitude_get(&L.att);
            const control_loop_stats_t *s = &L.stats;
            uint32_t avg = proc_n ? (uint32_t)(proc_sum / proc_n) : 0;
            fc_wdt_feed();   /* 試験アプリだけの保険: USBの詰まりで printf が長引いてもリセットさせない */
            printf("[%4us] %-9s estop=%s desc=%s guard=%s ctrl=%s att[deg] r=%+6.2f p=%+6.2f y=%+7.2f"
                   " thr=%.2f T=%.1fg duty=(%.3f %.3f %.3f %.3f)%s"
                   " proc_us avg/max=%u/%u over=%u TMO=%u RESC=%u MISS=%u ERR=%u STUCK=%u NOSMP=%u WDT_fed=%u"
                   " pc[conn=%d arm=%d pkts=%u] PWM=%s\n",
                   (unsigned)sec, flight_state_name(L.fsm.state), estop_reason_name(L.fsm.estop_reason),
                   descent_reason_name(L.fsm.descent_reason), takeover_reason_name(L.guard.reason),
                   L.policy_used ? "POLICY" : "PID",
                   (double)(as->angle.roll_rad * RAD_TO_DEG), (double)(as->angle.pitch_rad * RAD_TO_DEG),
                   (double)(as->angle.yaw_rad * RAD_TO_DEG),
                   (double)pc.throttle, (double)L.out.thrust_g,
                   (double)L.duty[0], (double)L.duty[1], (double)L.duty[2], (double)L.duty[3],
                   L.mix.saturated ? "(sat)" : "",
                   (unsigned)avg, (unsigned)proc_max, (unsigned)s->over_budget,
                   (unsigned)s->timeouts, (unsigned)s->rescues, (unsigned)s->miss,
                   (unsigned)s->read_errors, (unsigned)s->int1_stuck, (unsigned)s->no_sample,
                   (unsigned)s->wdt_feeds,
                   pc.connected ? 1 : 0, pc.arm ? 1 : 0, (unsigned)pc.sent, PWM_ENABLED ? "ON" : "OFF");
            fc_wdt_feed();
            proc_sum = 0; proc_max = 0; proc_n = 0;
            last_report_us = fc_clock_now_us32();
        }
    }
}
