/**
 * @file test_loop.c
 * @brief control_loop の試験。IMU・PWM・時計・ADC・WDT を偽物にして（mailbox / ipc_flags は本物）、
 *        1周期ずつ（control_loop_step）回して状態の流れと安全動作を確かめる。
 *
 * 偽物の時計は INT1 待ちが進める（1周期 1064µs、タイムアウトは5000µs）。
 * 「PC」の役は test 側: 50Hz(20ms)でパケットを mailbox_write する。
 * 毎周期の不変条件: 飛行中・降下中以外は motor_pwm_set を呼ばず、デューティは0。
 */
#include <stdlib.h>
#include "control_loop.h"
#include "fake_port.h"
#include "mailbox.h"
#include "ipc_flags.h"
#include "test_common.h"

/** 本物の mailbox / ipc_flags を空に戻す（試験の最初に）。 */
static void fake_ipc_reset(void)
{
    control_packet_t z;
    memset(&z, 0, sizeof(z));
    mailbox_write(&z);
    mailbox_test_force_seq(0);
    ipc_flags_init(ipc_flags());
}

static control_loop_t L;
static params_t LP;

static control_packet_t pkt;        /* 「PC」が送るパケット（内容が変わったら即送る。変わらなくても20msごと） */
static control_packet_t pkt_sent;
static bool pc_sending;
static uint32_t pc_next_us, pc_seq;
static int steps, motor_violations;

#define THR_G(thr)  ((thr) * 28.0f)                      /* 既定 throttle_full_thrust_g */
#define DUTY_FOR(g) ((g) / (9.4f * 3.8006f))             /* 電池 3.8V のとき */

static void setup(void)
{
    fake_port_reset();
    fake_ipc_reset();
    params_set_defaults(&LP);
    /* ループの試験は状態遷移を見るもの。傾きへの追従を速くして試験を短く保つため、τは既定値（2026-11-13 に 2.0s）
     * によらず 0.5s に固定する。τそのものの性質は test_main.c の姿勢推定の試験が既定値で見る */
    LP.attitude.tau_s = 0.5f;
    memset(&pkt, 0, sizeof(pkt)); memset(&pkt_sent, 0, sizeof(pkt_sent));
    pc_sending = false; pc_next_us = 0; pc_seq = 0; steps = 0; motor_violations = 0;
    fake_imu_gyro = (vec3f_t){ 0.01f, -0.02f, 0.005f };  /* 基板のゼロ点ずれ（バイアス推定の確認） */
    control_loop_init(&L, &LP);
}

static void pc_send_now(void)
{
    pkt.packet_seq = ++pc_seq;
    mailbox_write(&pkt);
    pkt_sent = pkt;
    pc_next_us = fake_now_us + 20000u;
}

/** 1周期。不変条件（停止状態でモーターが動かない）をここで毎回見る。 */
static void cycle(void)
{
    if (pc_sending && ((int32_t)(fake_now_us - pc_next_us) >= 0 ||
                       pkt.roll != pkt_sent.roll || pkt.pitch != pkt_sent.pitch || pkt.yaw != pkt_sent.yaw ||
                       pkt.throttle != pkt_sent.throttle || pkt.flags != pkt_sent.flags)) pc_send_now();
    int sets = fake_pwm_set_calls;
    control_loop_step(&L);
    steps++;
    /* 飛行中でもスロットルが idle_throttle 以下なら止める（提案）。そのときも「停止」と同じ条件で見る */
    const bool idle = L.fsm.state == FS_FLIGHT && L.target.throttle <= LP.safety.idle_throttle;
    if (!flight_fsm_motors_allowed(L.fsm.state) || idle) {
        bool zero = true;
        for (int i = 0; i < FC_NUM_MOTORS; i++) if (fake_pwm_duty[i] != 0.0f) zero = false;
        if (fake_pwm_set_calls != sets || !zero || L.motors_on) motor_violations++;
    } else {
        if (!L.motors_on) motor_violations++;
    }
}

static void run(int n) { for (int i = 0; i < n; i++) cycle(); }

/** 条件が成り立つまで最大 max 周期。成立した周期数（成立しなければ -1）。 */
#define RUN_UNTIL(cond, max) ({ int n_ = -1; for (int i_ = 0; i_ < (max); i_++) { cycle(); if (cond) { n_ = i_ + 1; break; } } n_; })

static void to_standby(void)
{
    setup();
    pc_sending = true; pc_next_us = fake_now_us;
    run(5);
}

static void to_ready(void)     /* 静止バイアス確定済みの待機 */
{
    to_standby();
    int n = RUN_UNTIL(attitude_bias_ready(&L.att), 3000);
    CHECK(n > 0 && n > 900);                            /* 静止1秒（約940周期）が必要 */
    run(20);
}

static void press_arm(void)    /* アーム旗を押す（押し続ける） */
{
    pkt.flags |= CTRL_FLAG_ARM_REQ;
    run(3);
}

static void to_flight(float throttle)
{
    to_ready();
    pkt.throttle = 0.0f;
    press_arm();
    pkt.throttle = throttle;
    run(5);
}

static void end_checks(void)
{
    CHECK(motor_violations == 0);                       /* 停止状態でモーターは常に0 */
    CHECK(fake_wdt_feeds == (uint32_t)steps);           /* WDTは毎周期（非常停止中も）1回 */
}

static void body_tilt_roll(float deg)   /* 機体を左側上げ方向に deg 傾けた加速度 */
{
    fake_imu_set_body_accel(0.0f, sinf(deg * DEG), cosf(deg * DEG));
}

/* ------------------------------------------------------------------ */

static void test_loop_idle_throttle(void)
{
    /* アームしたままスロットル0で傾けても、モーターは回らない（提案）。上げれば回る */
    setup();
    to_flight(0.0f);
    CHECK(L.fsm.state == FS_FLIGHT);
    body_tilt_roll(8.0f);
    run(200);
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK(!L.motors_on);
    pkt.throttle = 0.4f;
    run(5);
    CHECK(L.motors_on);
    pkt.throttle = 0.0f;
    run(5);
    CHECK(!L.motors_on);
    end_checks();
}

static void test_loop_init_ok(void)
{
    setup();
    CHECK(fake_pwm_init_calls == 1);
    CHECK(fake_pwm_stop_calls >= 1);                    /* 最初にPWMを0にしている */
    CHECK(fake_wdt_start_calls == 1);
    CHECK(fake_wdt_timeout_ms == LP.safety.wdt_timeout_ms);   /* WDTの時間はparamsから */
    CHECK(L.fsm.state == FS_WAIT_COMM);
    CHECK(L.fsm.estop_reason == ESTOP_NONE);
    CHECK(L.pid_ctrl.ops != NULL);
    CHECK(L.policy_ctrl.ops == NULL);                   /* 方策は未実装 = 使えない */
    CHECK(L.motor_output_enabled && L.motor_duty_max == 1.0f && !L.bench_no_battery);   /* 既定は本番の値 */
    CHECK(fake_batt_init_calls == 1);
    CHECK_NEAR(L.batt.voltage_v, 3.8f, 0.01f);
    end_checks();
}

static void test_loop_init_failures(void)
{
    /* IMU無応答 → 非常停止 */
    fake_port_reset(); fake_ipc_reset(); params_set_defaults(&LP);
    fake_imu_init_result = IMU_ERR_WHOAMI;
    control_loop_init(&L, &LP);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP); CHECK(L.fsm.estop_reason == ESTOP_IMU_INIT_FAIL);
    CHECK(fake_wdt_start_calls == 1);                   /* 非常停止でもWDTは開始（ループは動かし続ける） */
    steps = 0; motor_violations = 0; pc_sending = false;
    run(30);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP);
    end_checks();

    /* WDT再起動 → 非常停止。パケットとアーム旗が来ても抜けない */
    fake_port_reset(); fake_ipc_reset(); params_set_defaults(&LP);
    fake_wdt_reboot = true;
    control_loop_init(&L, &LP);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP); CHECK(L.fsm.estop_reason == ESTOP_WDT_REBOOT);
    memset(&pkt, 0, sizeof(pkt)); pkt.flags = CTRL_FLAG_ARM_REQ;
    steps = 0; motor_violations = 0; pc_sending = true; pc_next_us = fake_now_us; pc_seq = 0;
    run(1500);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP); CHECK(L.fsm.estop_reason == ESTOP_WDT_REBOOT);
    CHECK(L.log.state == FS_EMERGENCY_STOP && L.log.estop_reason == ESTOP_WDT_REBOOT);   /* ログは作り続ける */
    end_checks();
}

static void test_loop_lifecycle(void)
{
    setup();
    CHECK(L.fsm.state == FS_WAIT_COMM);

    /* 通信なしでは通信待機のまま */
    run(60);
    CHECK(L.fsm.state == FS_WAIT_COMM);
    CHECK(fake_pwm_set_calls == 0);

    /* 最初の有効パケット → 待機 */
    pc_sending = true; pc_next_us = fake_now_us;
    run(3);
    CHECK(L.fsm.state == FS_STANDBY);

    /* バイアス未確定のうちはアーム要求を見送る（提案）。旗を離して押し直すまで辺は来ない */
    pkt.flags = CTRL_FLAG_ARM_REQ; run(60);
    CHECK(L.fsm.state == FS_STANDBY);
    pkt.flags = 0; run(60);

    run(1100);                                          /* 静止が続いてバイアス確定 */
    CHECK(attitude_bias_ready(&L.att));
    CHECK(!L.att.bias_locked);                          /* 待機中は更新可 */

    /* スロットルが0でないとアームしない（辺は消費される） */
    pkt.throttle = 0.5f; pkt.flags = CTRL_FLAG_ARM_REQ; run(60);
    CHECK(L.fsm.state == FS_STANDBY);
    pkt.throttle = 0.0f; run(60);                       /* 旗を立てたまま0に: 辺は過ぎたのでアームしない */
    CHECK(L.fsm.state == FS_STANDBY);

    /* 離して、水平・スロットル0で押す → 飛行 */
    pkt.flags = 0; run(40);
    pkt.flags = CTRL_FLAG_ARM_REQ; run(3);
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK(L.att.bias_locked);                           /* アームでバイアス固定 */
    CHECK(L.active_controller == CONTROLLER_PID);
    CHECK(L.fsm.state_enter_us > 0);

    /* 飛行: スロットル0.3 → 1モーター 8.4g、水平なので4つとも同じデューティ */
    pkt.throttle = 0.3f; run(100);
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK(fake_pwm_set_calls > 0);
    for (int i = 0; i < FC_NUM_MOTORS; i++) CHECK_NEAR(fake_pwm_duty[i], DUTY_FOR(THR_G(0.3f)), 0.01f);
    CHECK_NEAR(L.target_thrust_g, 8.4f, 1e-4f);
    CHECK(L.log.flags & LOG_FLAG_MOTORS_ON);
    CHECK(L.log.state == FS_FLIGHT);

    /* ディスアーム（ipc_flags のカウンタ）→ 待機。モーター0。処理済みになる */
    ipc_flags_request_disarm(ipc_flags());
    CHECK(ipc_flags_disarm_pending(ipc_flags()));
    cycle();
    CHECK(L.fsm.state == FS_STANDBY);
    CHECK(!ipc_flags_disarm_pending(ipc_flags()));
    CHECK(!L.att.bias_locked);                          /* 待機に戻ったらバイアス更新を再開 */
    run(100);
    CHECK(L.fsm.state == FS_STANDBY);                   /* アーム旗は立ったまま: 辺が無いので再アームしない */

    /* 再アーム（離して押す）→ 飛行 → パケットのディスアーム旗 → 待機 */
    pkt.throttle = 0.0f; pkt.flags = 0; run(40);
    pkt.flags = CTRL_FLAG_ARM_REQ; run(3);
    CHECK(L.fsm.state == FS_FLIGHT);
    pkt.flags = CTRL_FLAG_ARM_REQ | CTRL_FLAG_DISARM; run(3);
    CHECK(L.fsm.state == FS_STANDBY);
    pkt.flags = CTRL_FLAG_DISARM; run(100);             /* ディスアーム旗が立っている間はアームしない */
    pkt.flags = CTRL_FLAG_DISARM | CTRL_FLAG_ARM_REQ; run(10);
    CHECK(L.fsm.state == FS_STANDBY);

    /* 待機中の通信途絶 → 通信待機（提案 #20）。再接続で待機へ */
    pkt.flags = 0; run(5);
    pc_sending = false; run(600);
    CHECK(L.fsm.state == FS_WAIT_COMM);
    pc_sending = true; pc_next_us = fake_now_us; run(5);
    CHECK(L.fsm.state == FS_STANDBY);
    end_checks();
}

static void test_loop_arm_conditions(void)
{
    /* 傾いていると辺があってもアームしない（許容10°）。水平に戻して押し直すとアームする */
    to_ready();
    body_tilt_roll(20.0f);
    run(1500);                                          /* 姿勢が約20°に収束。まだ60°未満なので非常停止にもならない */
    CHECK(attitude_tilt_rad(attitude_get(&L.att)) > 15.0f * DEG);
    pkt.flags = 0; run(5);
    pkt.flags = CTRL_FLAG_ARM_REQ; run(5);
    CHECK(L.fsm.state == FS_STANDBY);
    body_tilt_roll(0.0f);
    pkt.flags = 0; run(3000);                           /* バイアス再確定も含めて十分 */
    CHECK(attitude_tilt_rad(attitude_get(&L.att)) < 2.0f * DEG);
    pkt.flags = CTRL_FLAG_ARM_REQ; run(5);
    CHECK(L.fsm.state == FS_FLIGHT);
    end_checks();

    /* 接続前から立っていた旗では、接続後すぐにアームしない（辺が必要） */
    setup();
    pkt.flags = CTRL_FLAG_ARM_REQ;
    pc_sending = true; pc_next_us = fake_now_us;
    run(1200);
    CHECK(L.fsm.state == FS_STANDBY);
    end_checks();

    /* 電池が低下していたらアームしない（提案） */
    to_ready();
    fake_batt_raw = 1861;                               /* 3.0V */
    run(2500);
    CHECK(L.batt.low);
    pkt.flags = 0; run(5); pkt.flags = CTRL_FLAG_ARM_REQ; run(5);
    CHECK(L.fsm.state == FS_STANDBY);
    end_checks();
}

static void test_loop_comm_loss_descent_landing(void)
{
    to_flight(0.3f);
    run(3000);                                          /* ホバリング推定が収束するまで */
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK_NEAR(L.descent.hover_thrust_est_g, 8.4f, 0.1f);

    /* パケットが止まる → comm_timeout_ms(500ms) のあと降下 */
    pc_sending = false;
    uint32_t t0 = fake_now_us;
    int n = RUN_UNTIL(L.fsm.state == FS_EMERGENCY_DESCENT, 800);
    CHECK(n > 0);
    uint32_t dt_ms = (fake_now_us - t0) / 1000u;
    CHECK(dt_ms >= LP.safety.comm_timeout_ms - 25u && dt_ms <= LP.safety.comm_timeout_ms + 25u);
    CHECK(L.fsm.descent_reason == DESCENT_COMM_LOST);
    CHECK(L.fsm.estop_reason == ESTOP_NONE);

    /* 降下: 推力は 比×ホバリング、PIDで水平を保つ。制御器はPID */
    run(100);
    CHECK(L.fsm.state == FS_EMERGENCY_DESCENT);
    CHECK_NEAR(L.target_thrust_g, LP.descent.thrust_ratio * 8.4f, 0.15f);
    for (int i = 0; i < FC_NUM_MOTORS; i++) CHECK_NEAR(fake_pwm_duty[i], DUTY_FOR(LP.descent.thrust_ratio * 8.4f), 0.015f);
    CHECK(!L.policy_used);
    CHECK(L.log.descent_reason == DESCENT_COMM_LOST && (L.log.flags & LOG_FLAG_MOTORS_ON));

    /* 通信が戻っても降下のまま（人に返さない） */
    pc_sending = true; pc_next_us = fake_now_us; pkt.flags = 0; run(60);
    CHECK(L.fsm.state == FS_EMERGENCY_DESCENT);
    /* ディスアーム旗も降下中は無視（機体が自分で降りる）。ipc のカウンタは処理済みにして残さない */
    ipc_flags_request_disarm(ipc_flags());
    run(3);
    CHECK(L.fsm.state == FS_EMERGENCY_DESCENT);
    CHECK(!ipc_flags_disarm_pending(ipc_flags()));

    /* 着地: 衝撃のあと静かな状態が 500ms → 非常停止（理由: 着地）。降下の原因も残る */
    fake_imu_set_body_accel(0.0f, 0.0f, 5.0f); run(4);
    CHECK(L.fsm.state == FS_EMERGENCY_DESCENT);
    fake_imu_set_body_accel(0.0f, 0.0f, 1.0f);
    uint32_t tq = fake_now_us;
    int m = RUN_UNTIL(L.fsm.state == FS_EMERGENCY_STOP, 800);
    CHECK(m > 0);
    uint32_t q_ms = (fake_now_us - tq) / 1000u;
    CHECK(q_ms >= LP.descent.quiet_hold_ms - 5u && q_ms <= LP.descent.quiet_hold_ms + 5u);   /* 静かさ500ms保持 */
    CHECK(L.fsm.estop_reason == ESTOP_DESCENT_LANDED);
    CHECK(L.fsm.descent_reason == DESCENT_COMM_LOST);
    run(500);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP);
    CHECK(L.log.state == FS_EMERGENCY_STOP && L.log.estop_reason == ESTOP_DESCENT_LANDED &&
          L.log.descent_reason == DESCENT_COMM_LOST);
    end_checks();
}

static void test_loop_descent_timeout(void)
{
    to_flight(0.3f);
    run(500);
    pc_sending = false;
    int n = RUN_UNTIL(L.fsm.state == FS_EMERGENCY_DESCENT, 800);
    CHECK(n > 0);
    uint32_t t0 = fake_now_us;
    /* 衝撃なし（台上と同じ）: 静かでも着地とは判定せず、上限時間で止まる */
    int m = RUN_UNTIL(L.fsm.state == FS_EMERGENCY_STOP, 4000);
    CHECK(m > 0);
    uint32_t dt_ms = (fake_now_us - t0) / 1000u;
    CHECK(dt_ms >= LP.descent.max_time_ms - 5u && dt_ms <= LP.descent.max_time_ms + 5u);
    CHECK(L.fsm.estop_reason == ESTOP_DESCENT_TIMEOUT);
    CHECK(L.fsm.descent_reason == DESCENT_COMM_LOST);
    run(200);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP);
    end_checks();
}

static void test_loop_battery_low_descent(void)
{
    to_flight(0.3f);
    run(200);
    fake_batt_raw = 1861;                               /* 3.0V に低下（ローパス1s + 保持0.5s で確定） */
    int n = RUN_UNTIL(L.fsm.state == FS_EMERGENCY_DESCENT, 3000);
    CHECK(n > 0);
    CHECK(L.fsm.descent_reason == DESCENT_BATTERY_LOW);
    run(50);
    CHECK(L.fsm.state == FS_EMERGENCY_DESCENT);
    end_checks();

    /* 電池なしの試験モード: 低下を無視して 3.8V 固定で換算 */
    to_ready();
    L.bench_no_battery = true;
    fake_batt_raw = 0;
    pkt.flags = 0; run(3); pkt.flags = CTRL_FLAG_ARM_REQ; run(3);
    CHECK(L.fsm.state == FS_FLIGHT);
    pkt.throttle = 0.3f; run(2500);
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK_NEAR(L.batt_v_used, 3.8f, 1e-6f);
    CHECK_NEAR(fake_pwm_duty[0], 8.4f / (9.4f * 3.8f), 0.01f);
    end_checks();
}

static void test_loop_operator_estop(void)
{
    /* どの状態からでも、ipc_flags の旗でもパケットの旗でも非常停止（理由: operator） */
    for (int via = 0; via < 2; via++) {
        for (int from = 0; from < 4; from++) {
            switch (from) {
            case 0: setup(); run(5); CHECK(L.fsm.state == FS_WAIT_COMM); break;
            case 1: to_standby(); CHECK(L.fsm.state == FS_STANDBY); break;
            case 2: to_flight(0.3f); CHECK(L.fsm.state == FS_FLIGHT); break;
            default:
                to_flight(0.3f); run(300); pc_sending = false;
                CHECK(RUN_UNTIL(L.fsm.state == FS_EMERGENCY_DESCENT, 800) > 0);
                break;
            }
            if (via == 0) ipc_flags_request_estop(ipc_flags());
            else { pc_sending = true; pc_next_us = fake_now_us; pkt.flags |= CTRL_FLAG_ESTOP; }
            run(3);
            CHECK(L.fsm.state == FS_EMERGENCY_STOP);
            CHECK(L.fsm.estop_reason == ESTOP_OPERATOR);
            run(1200);
            CHECK(L.fsm.state == FS_EMERGENCY_STOP);   /* 旗を戻しても、アーム旗を立てても抜けない */
            pkt.flags = CTRL_FLAG_ARM_REQ; pc_sending = true; run(100); pkt.flags = 0; run(100);
            pkt.flags = CTRL_FLAG_ARM_REQ; run(100);
            CHECK(L.fsm.state == FS_EMERGENCY_STOP);
            end_checks();
        }
    }
}

static void test_loop_tilt_estop(void)
{
    to_flight(0.3f);
    body_tilt_roll(75.0f);
    int n = RUN_UNTIL(L.fsm.state == FS_EMERGENCY_STOP, 3000);
    CHECK(n > 0);
    CHECK(L.fsm.estop_reason == ESTOP_TILT_EXCEEDED);
    CHECK(L.fsm.descent_reason == DESCENT_NONE);        /* 降下を経ずに止まる */
    CHECK(attitude_tilt_rad(attitude_get(&L.att)) > 60.0f * DEG);
    body_tilt_roll(0.0f);
    run(1500);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP);           /* 水平に戻っても抜けない */
    end_checks();

    /* 50° では止まらない */
    to_flight(0.3f);
    body_tilt_roll(50.0f);
    run(3000);
    CHECK(L.fsm.state == FS_FLIGHT);
    end_checks();

    /* 降下中の傾き過大も非常停止（提案） */
    to_flight(0.3f);
    pc_sending = false;
    CHECK(RUN_UNTIL(L.fsm.state == FS_EMERGENCY_DESCENT, 800) > 0);
    body_tilt_roll(75.0f);
    CHECK(RUN_UNTIL(L.fsm.state == FS_EMERGENCY_STOP, 3000) > 0);
    CHECK(L.fsm.estop_reason == ESTOP_TILT_EXCEEDED && L.fsm.descent_reason == DESCENT_COMM_LOST);
    end_checks();
}

static void test_loop_imu_timeout(void)
{
    /* 飛行中にINT1が来ずピンもLow → 即、モーター停止・非常停止。時間基準(5ms)で回り続け、WDTも餌やり */
    to_flight(0.3f);
    CHECK(fake_pwm_set_calls > 0);
    fake_imu_wait_timeout = true; fake_imu_pin_high = false;
    uint32_t now0 = fake_now_us;
    cycle();
    CHECK(L.fsm.state == FS_EMERGENCY_STOP); CHECK(L.fsm.estop_reason == ESTOP_IMU_TIMEOUT);
    CHECK(L.stats.timeouts == 1); CHECK(L.stats.rescues == 0);
    CHECK(L.imu_stalled);
    run(19);
    CHECK((uint32_t)(fake_now_us - now0) == 20u * FC_INT1_TIMEOUT_US);   /* 1周期=5ms の時間基準 */
    CHECK(L.fsm.state == FS_EMERGENCY_STOP);
    CHECK(L.log.state == FS_EMERGENCY_STOP && L.log.estop_reason == ESTOP_IMU_TIMEOUT);
    CHECK(!L.int1_pending);
    /* IMUが戻っても非常停止のまま、ログは作り続ける */
    fake_imu_wait_timeout = false;
    run(50);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP);
    end_checks();

    /* 提案: 通信待機・待機でもIMU途絶は非常停止 */
    setup(); fake_imu_wait_timeout = true; cycle();
    CHECK(L.fsm.state == FS_EMERGENCY_STOP && L.fsm.estop_reason == ESTOP_IMU_TIMEOUT);
    end_checks();
    to_standby(); fake_imu_wait_timeout = true; cycle();
    CHECK(L.fsm.state == FS_EMERGENCY_STOP && L.fsm.estop_reason == ESTOP_IMU_TIMEOUT);
    end_checks();

    /* 降下中のIMU途絶も非常停止 */
    to_flight(0.3f); pc_sending = false;
    CHECK(RUN_UNTIL(L.fsm.state == FS_EMERGENCY_DESCENT, 800) > 0);
    fake_imu_wait_timeout = true; cycle();
    CHECK(L.fsm.state == FS_EMERGENCY_STOP && L.fsm.estop_reason == ESTOP_IMU_TIMEOUT);
    CHECK(L.fsm.descent_reason == DESCENT_COMM_LOST);
    end_checks();

    /* タイムアウトでもピンがHighなら救出（読んで続ける）。非常停止にしない */
    to_flight(0.3f);
    fake_imu_wait_timeout = true; fake_imu_pin_high = true;
    int reads = fake_imu_read_calls;
    cycle();
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK(L.stats.timeouts == 1); CHECK(L.stats.rescues == 1);
    CHECK(fake_imu_read_calls == reads + 1);
    CHECK(!L.imu_stalled);
    end_checks();

    /* 読み出し失敗が連続3回でIMU途絶。1〜2回は古い姿勢で続ける */
    to_flight(0.3f);
    fake_imu_read_result = IMU_ERR_SPI;
    fake_imu_pin_high = true;                           /* 失敗した周期は急いで読み直さない */
    cycle(); CHECK(L.fsm.state == FS_FLIGHT); CHECK(!L.int1_pending);
    cycle(); CHECK(L.fsm.state == FS_FLIGHT);
    cycle(); CHECK(L.fsm.state == FS_EMERGENCY_STOP); CHECK(L.fsm.estop_reason == ESTOP_IMU_TIMEOUT);
    CHECK(L.stats.read_errors == 3);
    end_checks();

    /* 失敗が散発なら（成功で数え直し）止まらない */
    to_flight(0.3f);
    for (int i = 0; i < 10; i++) {
        fake_imu_read_result = IMU_ERR_SPI; cycle(); cycle();
        fake_imu_read_result = IMU_OK; cycle();
    }
    CHECK(L.fsm.state == FS_FLIGHT);
    end_checks();
}

static void test_loop_missed_edge(void)
{
    to_flight(0.3f);
    /* 読み終えてもINT1がHigh → 戻り値 true、次は待たずに読む */
    imu_int1_event_t ev = { fake_imu_int1_count + 1u, fake_now_us };
    fake_imu_pin_high = true;
    bool pend = control_loop_run_once(&L, &ev);
    steps++;
    CHECK(pend); CHECK(L.int1_pending); CHECK(L.stats.miss == 1);
    int waits = fake_imu_wait_calls, reads = fake_imu_read_calls;
    cycle();                                            /* 待たずに読む */
    CHECK(fake_imu_wait_calls == waits); CHECK(fake_imu_read_calls == reads + 1);
    CHECK(L.int1_pending); CHECK(L.stats.miss == 2);
    fake_imu_pin_high = false;
    cycle();                                            /* まだ待たない（pendingだったので）。今度はLowなので解除 */
    CHECK(fake_imu_wait_calls == waits); CHECK(!L.int1_pending);
    cycle();                                            /* 通常に戻る */
    CHECK(fake_imu_wait_calls == waits + 1);
    CHECK(L.fsm.state == FS_FLIGHT);
    end_checks();
}

static void test_loop_log_and_timing(void)
{
    to_flight(0.3f);
    cycle();
    const log_record_t *r = &L.log;
    CHECK(r->state == FS_FLIGHT);
    CHECK(r->flags & LOG_FLAG_MOTORS_ON);
    CHECK(r->proc_us == fake_imu_read_cost_us);         /* 偽物の時計は読み出しだけが進める。ログ組み立ては含まない */
    CHECK(r->wake_latency_us == 0);
    CHECK(r->fifo_words == 16);
    CHECK(r->battery_mv > 3700 && r->battery_mv < 3900);
    CHECK_NEAR(r->target_thrust, 8.4f, 1e-4f);
    CHECK_NEAR(r->output[0], 8.4f, 1e-4f);
    for (int i = 0; i < 4; i++) CHECK_NEAR(r->duty[i], fake_pwm_duty[i], 1e-6f);
    CHECK_NEAR(r->attitude[0], 0.0f, 0.05f);
    CHECK(L.timing.cycle_index == (uint32_t)steps);
    CHECK(L.stats.cycles == (uint32_t)steps);
    CHECK(L.stats.over_budget == 0);

    fake_imu_read_cost_us = 1200;                       /* 周期を超える処理 */
    cycle();
    CHECK(L.stats.over_budget == 1);
    CHECK(L.log.proc_us == 1200);

    /* 取りこぼし: INT1の回数が2進んだら int1_missed=1 */
    fake_imu_read_cost_us = 160;
    fake_imu_int1_count += 1;
    cycle();
    CHECK(L.log.int1_missed == 1);
    end_checks();
}

static void test_loop_pwm_safety_switches(void)
{
    /* 出力を無効にすると、飛行中でもピンは0のまま（デューティは計算してログに出す） */
    to_ready();
    L.motor_output_enabled = false;
    pkt.flags = 0; run(3); pkt.flags = CTRL_FLAG_ARM_REQ; run(3);
    CHECK(L.fsm.state == FS_FLIGHT);
    pkt.throttle = 0.5f;
    int sets = fake_pwm_set_calls;
    run(50);
    CHECK(fake_pwm_set_calls == sets);
    for (int i = 0; i < 4; i++) CHECK(fake_pwm_duty[i] == 0.0f);
    CHECK(L.duty[0] > 0.2f);
    CHECK(L.log.duty[0] > 0.2f);
    CHECK(L.motors_on);                                 /* 状態としては飛行中 */
    motor_violations = 0;                               /* この試験では motors_on の意味が違うので数え直し */
    end_checks();

    /* デューティの上限 */
    to_ready();
    L.motor_duty_max = 0.10f;
    pkt.flags = 0; run(3); pkt.flags = CTRL_FLAG_ARM_REQ; run(3);
    pkt.throttle = 1.0f; run(50);
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK(L.duty[0] > 0.3f);
    for (int i = 0; i < 4; i++) CHECK(fake_pwm_duty[i] <= 0.10f + 1e-6f && fake_pwm_duty[i] > 0.0f);
    end_checks();
}

/* ---------------- 方策（偽物）と見張り役 ---------------- */

static struct {
    controller_output_t out;   /* follow==false のときの出力 */
    bool follow;               /* true: 目標推力+1g、トルク0 */
    bool ok;
    uint32_t cost_us;
    int calls, resets;
} FP;

static void fp_reset(void *self) { (void)self; FP.resets++; }
static bool fp_step(void *self, const controller_input_t *in, float dt, controller_output_t *out)
{
    (void)self; (void)dt;
    FP.calls++;
    fake_now_us += FP.cost_us;
    if (FP.follow) { out->thrust_g = in->target_thrust_g + 1.0f; out->torque_g[0] = out->torque_g[1] = out->torque_g[2] = 0.0f; }
    else *out = FP.out;
    return FP.ok;
}
static const controller_ops_t FP_OPS = { "fake_policy", CONTROLLER_POLICY, fp_reset, fp_step };

static void fp_install(void)
{
    memset(&FP, 0, sizeof(FP));
    FP.follow = true; FP.ok = true; FP.cost_us = 100;
    controller_t h = { &FP_OPS, &FP };
    control_loop_set_policy(&L, h);
}

/** 方策を選んでアームし、飛行に入る（待機から）。 */
static void policy_flight(void)
{
    to_ready();
    fp_install();
    CHECK(control_loop_select_controller(&L, CONTROLLER_POLICY));
    pkt.throttle = 0.0f; pkt.flags = 0; run(3);
    pkt.flags = CTRL_FLAG_ARM_REQ; run(3);
    pkt.throttle = 0.3f; run(20);
}

static void test_loop_policy_unavailable(void)
{
    /* 方策を選んでもハンドルが無ければPID（今の実装状態） */
    to_ready();
    CHECK(control_loop_select_controller(&L, CONTROLLER_POLICY));
    pkt.flags = 0; run(3); pkt.flags = CTRL_FLAG_ARM_REQ; run(3);
    pkt.throttle = 0.3f; run(20);
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK(L.selected_controller == CONTROLLER_POLICY);
    CHECK(L.active_controller == CONTROLLER_PID);
    CHECK(!L.policy_used); CHECK(L.log.active_controller == CONTROLLER_PID);
    CHECK_NEAR(L.out.thrust_g, 8.4f, 1e-3f);
    /* 選択は待機のときだけ */
    CHECK(!control_loop_select_controller(&L, CONTROLLER_PID));
    CHECK(!control_loop_params_change_allowed(&L));
    end_checks();
}

static void test_loop_policy_normal_and_latch(void)
{
    policy_flight();
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK(L.active_controller == CONTROLLER_POLICY);
    CHECK(FP.resets >= 1);                              /* アームで方策もリセット */
    CHECK(L.policy_used); CHECK(L.log.flags & LOG_FLAG_POLICY_ACTIVE);
    CHECK_NEAR(L.out.thrust_g, 9.4f, 1e-3f);            /* 方策の出力（目標+1g）を採用 */
    CHECK(L.policy_us == 100 && L.log.policy_us == 100);
    CHECK(!guard_taken_over(&L.guard));
    CHECK_NEAR(L.pid_shadow_out.thrust_g, 8.4f, 1e-3f); /* 影のPIDも毎周期回っている */

    /* 傾きがやや大きい(30°超)でPIDが取り上げる */
    body_tilt_roll(40.0f);
    int n = RUN_UNTIL(guard_taken_over(&L.guard), 2000);
    CHECK(n > 0);
    CHECK(L.guard.reason == TAKEOVER_TILT);
    CHECK(!L.policy_used);
    CHECK_NEAR(L.out.thrust_g, 8.4f, 1e-3f);           /* 同じ周期からPIDの出力 */
    CHECK(L.log.takeover_reason == TAKEOVER_TILT);
    CHECK(L.fsm.state == FS_FLIGHT);

    /* 取り上げたら、水平に戻っても方策には戻さない。方策はもう呼ばない */
    int calls = FP.calls;
    body_tilt_roll(0.0f);
    run(2500);
    CHECK(L.fsm.state == FS_FLIGHT);
    CHECK(guard_taken_over(&L.guard)); CHECK(!L.policy_used);
    CHECK(FP.calls == calls);

    /* 待機に戻して選び直すと解除され、次のアームで再び方策 */
    ipc_flags_request_disarm(ipc_flags()); run(2);
    CHECK(L.fsm.state == FS_STANDBY);
    CHECK(guard_taken_over(&L.guard));                  /* 待機に戻っただけではまだ（選び直しで解除） */
    CHECK(control_loop_select_controller(&L, CONTROLLER_POLICY));
    CHECK(!guard_taken_over(&L.guard));
    pkt.throttle = 0.0f; pkt.flags = 0; run(3000);      /* 水平・静止を再確認 */
    pkt.flags = CTRL_FLAG_ARM_REQ; run(3);
    CHECK(L.fsm.state == FS_FLIGHT);
    pkt.throttle = 0.3f; run(10);
    CHECK(L.policy_used); CHECK(!guard_taken_over(&L.guard));
    end_checks();
}

static void test_loop_policy_takeover_reasons(void)
{
    /* 人の指示（パケットの旗） */
    policy_flight();
    pkt.flags |= CTRL_FLAG_TAKEOVER_PID; run(3);
    CHECK(L.guard.reason == TAKEOVER_HUMAN); CHECK(!L.policy_used);
    end_checks();

    /* 角速度が大きすぎる（ヨー軸 9rad/s > 8） */
    policy_flight();
    fake_imu_set_body_gyro(0.0f, 0.0f, 9.0f); run(3);
    CHECK(L.guard.reason == TAKEOVER_RATE); CHECK(!L.policy_used);
    end_checks();

    /* 出力がNaN: その周期からPIDの出力（モーターにNaNを出さない） */
    policy_flight();
    FP.follow = false; FP.out = (controller_output_t){ 9.0f, { NAN, 0.0f, 0.0f } };
    cycle();
    CHECK(L.guard.reason == TAKEOVER_POLICY_NAN); CHECK(!L.policy_used);
    for (int i = 0; i < 4; i++) CHECK(isfinite(fake_pwm_duty[i]) && fake_pwm_duty[i] > 0.0f);
    CHECK_NEAR(L.out.thrust_g, 8.4f, 1e-3f);
    end_checks();

    /* 計算時間が予算(600µs)超過 */
    policy_flight();
    FP.cost_us = LP.guard.policy_budget_us + 1u; cycle();
    CHECK(L.guard.reason == TAKEOVER_POLICY_OVERRUN); CHECK(!L.policy_used);
    end_checks();

    /* 上限に張りつきが policy_sat_cycles 周期続く */
    policy_flight();
    FP.follow = false; FP.out = (controller_output_t){ 9.0f, { 6.0f, 0.0f, 0.0f } };
    for (unsigned i = 0; i + 1 < LP.guard.policy_sat_cycles; i++) cycle();
    CHECK(L.guard.reason == TAKEOVER_NONE); CHECK(L.policy_used);
    cycle();
    CHECK(L.guard.reason == TAKEOVER_POLICY_SATURATED); CHECK(!L.policy_used);
    end_checks();

    /* 方策の step が失敗（重み未ロードなど） */
    policy_flight();
    FP.ok = false; cycle();
    CHECK(L.guard.reason == TAKEOVER_POLICY_FAIL); CHECK(!L.policy_used);
    end_checks();
}

static void test_loop_policy_not_used_in_descent(void)
{
    /* 降下は必ずPID: 方策で飛んでいても通信が切れたら方策を呼ばない */
    policy_flight();
    run(200);
    pc_sending = false;
    CHECK(RUN_UNTIL(L.fsm.state == FS_EMERGENCY_DESCENT, 800) > 0);
    int calls = FP.calls;
    run(100);
    CHECK(FP.calls == calls); CHECK(!L.policy_used);
    CHECK(L.log.active_controller == CONTROLLER_PID);
    end_checks();
}


/* ---- 提案: サンプル0個の周期は制御を進めない／INT1張り付きの検出 ---- */

static void test_loop_int1_stuck_latch(void)
{
    /* ピンがHighのまま、読むと0個。8周期で張り付きをラッチ → IMU途絶の非常停止、読み直しも止まる */
    to_standby();
    fake_imu_pin_high = true; fake_imu_read_samples = 0;
    int n = 0;
    for (int i = 0; i < (int)FC_LOOP_INT1_STUCK_LIMIT - 1; i++) {
        imu_int1_event_t ev = { ++fake_imu_int1_count, fake_now_us };
        bool pend = control_loop_run_once(&L, &ev); steps++; n++;
        CHECK(pend); CHECK(!L.int1_stuck);
    }
    imu_int1_event_t ev = { ++fake_imu_int1_count, fake_now_us };
    bool pend = control_loop_run_once(&L, &ev); steps++;
    CHECK(!pend); CHECK(!L.int1_pending);
    CHECK(L.int1_stuck); CHECK(L.stats.int1_stuck == 1);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP); CHECK(L.fsm.estop_reason == ESTOP_IMU_TIMEOUT);
    CHECK(L.stats.no_sample == FC_LOOP_INT1_STUCK_LIMIT);
    /* ピンが戻って読み出しが正常になってもラッチは残る */
    fake_imu_pin_high = false; fake_imu_read_samples = 8;
    run(50);
    CHECK(L.int1_stuck); CHECK(L.stats.int1_stuck == 1);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP);
    end_checks();
}

static void test_loop_int1_genuine_miss_no_latch(void)
{
    /* ピンがHighでも8組(16ワード)読めているなら本物の取りこぼし。ラッチしない */
    to_flight(0.3f);
    fake_imu_pin_high = true; fake_imu_read_samples = 8;
    for (int i = 0; i < (int)FC_LOOP_INT1_STUCK_LIMIT * 5; i++) {
        imu_int1_event_t ev = { ++fake_imu_int1_count, fake_now_us };
        control_loop_run_once(&L, &ev); steps++;
    }
    CHECK(!L.int1_stuck); CHECK(L.int1_stuck_count == 0); CHECK(L.stats.int1_stuck == 0);
    CHECK(L.fsm.state == FS_FLIGHT);
    /* 散発の疑わしい周期は、正常な周期で数え直し */
    for (int k = 0; k < 5; k++) {
        fake_imu_read_samples = 0;
        for (int i = 0; i < (int)FC_LOOP_INT1_STUCK_LIMIT - 1; i++) {
            imu_int1_event_t ev = { ++fake_imu_int1_count, fake_now_us };
            control_loop_run_once(&L, &ev); steps++;
        }
        fake_imu_read_samples = 8;
        imu_int1_event_t ev = { ++fake_imu_int1_count, fake_now_us };
        control_loop_run_once(&L, &ev); steps++;
    }
    CHECK(!L.int1_stuck);
    end_checks();
}

static void test_loop_hold_no_samples(void)
{
    /* 飛行中にサンプル0個の周期: PIDを進めず、前回の指令を保つ */
    to_flight(0.4f);
    body_tilt_roll(3.0f);
    run(30);
    CHECK(L.fsm.state == FS_FLIGHT && L.motors_on);
    controller_output_t prev = L.out;
    float duty[FC_NUM_MOTORS];
    for (int i = 0; i < FC_NUM_MOTORS; i++) duty[i] = L.duty[i];
    pid_controller_t pid = L.pid;
    controller_output_t shadow = L.pid_shadow_out;
    uint32_t ns = L.stats.no_sample;
    fake_imu_read_samples = 0;
    cycle();
    CHECK(L.stats.no_sample == ns + 1);
    CHECK(L.fsm.state == FS_FLIGHT); CHECK(L.motors_on);
    CHECK(memcmp(&L.pid, &pid, sizeof(pid)) == 0);      /* 積分・フィルタとも不変 */
    CHECK(memcmp(&L.out, &prev, sizeof(prev)) == 0);
    CHECK(memcmp(&L.pid_shadow_out, &shadow, sizeof(shadow)) == 0);
    for (int i = 0; i < FC_NUM_MOTORS; i++) { CHECK(L.duty[i] == duty[i]); CHECK(fake_pwm_duty[i] == duty[i]); }
    CHECK(L.policy_us == 0);
    end_checks();
}

static void test_loop_hold_read_error(void)
{
    /* 読み出し失敗が1周期だけなら前回の指令を保ち、非常停止にしない（上限3） */
    to_flight(0.4f);
    body_tilt_roll(3.0f);
    run(30);
    controller_output_t prev = L.out;
    float duty[FC_NUM_MOTORS];
    for (int i = 0; i < FC_NUM_MOTORS; i++) duty[i] = L.duty[i];
    pid_controller_t pid = L.pid;
    fake_imu_read_result = IMU_ERR_SPI;
    cycle();
    CHECK(L.fsm.state == FS_FLIGHT); CHECK(L.motors_on);
    CHECK(memcmp(&L.pid, &pid, sizeof(pid)) == 0);
    CHECK(memcmp(&L.out, &prev, sizeof(prev)) == 0);
    for (int i = 0; i < FC_NUM_MOTORS; i++) CHECK(L.duty[i] == duty[i]);
    fake_imu_read_result = IMU_OK;
    run(10);
    CHECK(L.fsm.state == FS_FLIGHT);
    end_checks();
}

static void test_loop_stuck_no_busy_spin(void)
{
    /* control_loop_step 経由: 張り付きでも読み出し回数はラッチ後に増えない（ビジー回りしない） */
    to_standby();
    fake_imu_pin_high = true; fake_imu_read_samples = 0;
    run((int)FC_LOOP_INT1_STUCK_LIMIT + 2);
    CHECK(L.int1_stuck); CHECK(!L.int1_pending);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP && L.fsm.estop_reason == ESTOP_IMU_TIMEOUT);
    int reads = fake_imu_read_calls;
    fake_imu_wait_timeout = true;                       /* 5ms待ち→タイムアウト（ピンはHighのまま）*/
    run(100);
    CHECK(fake_imu_read_calls == reads);                /* 救出の読み出しもしない */
    CHECK(L.stats.rescues == 0);
    CHECK(L.fsm.state == FS_EMERGENCY_STOP);
    end_checks();
}

static void test_loop_no_sample_limit(void)
{
    /* 提案: ピンはLowでINT1は来るのにサンプル0個が続く。保持は FC_LOOP_NO_SAMPLE_LIMIT 周期までで、IMU途絶の非常停止 */
    to_flight(0.4f);
    run(30);
    fake_imu_read_samples = 0;
    for (int i = 0; i < (int)FC_LOOP_NO_SAMPLE_LIMIT - 1; i++) {
        cycle();
        CHECK(L.fsm.state == FS_FLIGHT); CHECK(L.motors_on);
    }
    CHECK(!L.int1_stuck);
    cycle();
    CHECK(L.fsm.state == FS_EMERGENCY_STOP); CHECK(L.fsm.estop_reason == ESTOP_IMU_TIMEOUT);
    CHECK(!L.motors_on);
    for (int i = 0; i < FC_NUM_MOTORS; i++) CHECK(fake_pwm_duty[i] == 0.0f);
    end_checks();
}

const test_t g_tests_loop[] = {
    { "loop: 起動（PWM0→初期化→WDT開始、既定は本番の値）", test_loop_init_ok },
    { "loop: 飛行中でもスロットル0付近では回さない（提案）", test_loop_idle_throttle },
    { "loop: 起動時の失敗（IMU無応答・WDT再起動）は非常停止", test_loop_init_failures },
    { "loop: 通信待機→待機→アーム→飛行→ディスアーム（フラグ/カウンタ）", test_loop_lifecycle },
    { "loop: アーム条件（水平・辺・電池）", test_loop_arm_conditions },
    { "loop: 通信途絶→降下→着地→非常停止", test_loop_comm_loss_descent_landing },
    { "loop: 通信途絶→降下→時間切れ→非常停止", test_loop_descent_timeout },
    { "loop: 電池低下→降下／電池なし試験モード", test_loop_battery_low_descent },
    { "loop: 操作者の非常停止（全状態×旗2種、抜けない）", test_loop_operator_estop },
    { "loop: 傾き60°超で非常停止", test_loop_tilt_estop },
    { "loop: IMU途絶（タイムアウト・救出・連続失敗）", test_loop_imu_timeout },
    { "loop: INT1の取りこぼし（ピンがHighのまま）", test_loop_missed_edge },
    { "loop: ログと周期時間", test_loop_log_and_timing },
    { "loop: 試験用スイッチ（PWM出力無効・デューティ上限）", test_loop_pwm_safety_switches },
    { "loop: 方策が使えないときはPID", test_loop_policy_unavailable },
    { "loop: 方策の使用・取り上げ・ラッチ・選び直し", test_loop_policy_normal_and_latch },
    { "loop: 取り上げの各原因（人・角速度・NaN・時間・張りつき・失敗）", test_loop_policy_takeover_reasons },
    { "loop: 降下では方策を呼ばない", test_loop_policy_not_used_in_descent },
    { "loop: INT1張り付きでラッチ→IMU途絶の非常停止（提案）", test_loop_int1_stuck_latch },
    { "loop: 本物の取りこぼしはラッチしない", test_loop_int1_genuine_miss_no_latch },
    { "loop: 飛行中サンプル0個の周期は前回の指令を保つ（提案）", test_loop_hold_no_samples },
    { "loop: 飛行中の読み出し失敗1回は指令を保つ", test_loop_hold_read_error },
    { "loop: 張り付き後はビジー回りしない（step経由）", test_loop_stuck_no_busy_spin },
    { "loop: サンプル0個が続いたらIMU途絶（提案）", test_loop_no_sample_limit },
};
const int g_tests_loop_n = (int)(sizeof(g_tests_loop) / sizeof(g_tests_loop[0]));
