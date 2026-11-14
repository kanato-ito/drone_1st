/**
 * @file test_fsm.c
 * @brief flight_state の遷移の試験（純粋関数なので入力を作って結果を見るだけ）。
 */
#include <stdlib.h>
#include "flight_state.h"
#include "test_common.h"

#define TILT_ESTOP (60.0f * DEG)

static flight_fsm_inputs_t base(void)
{
    flight_fsm_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.boot_imu_ok = true;
    return in;
}

/** 状態 s から1回進める。結果の fsm を f に入れ、changed を返す。 */
static bool run_from(flight_state_t s, const flight_fsm_inputs_t *in, flight_fsm_t *f)
{
    flight_fsm_init(f, 0);
    f->state = s;
    return flight_fsm_step(f, in, TILT_ESTOP, 1000);
}

/** 期待: 状態 want、changed、停止理由。 */
#define EXPECT(from, in, want, want_changed, want_reason) do { \
    flight_fsm_t f_; bool ch_ = run_from((from), &(in), &f_); \
    CHECK(f_.state == (want)); CHECK(ch_ == (want_changed)); CHECK(f_.estop_reason == (want_reason)); } while (0)

static void test_fsm_boot(void)
{
    flight_fsm_inputs_t in = base();
    EXPECT(FS_BOOT, in, FS_WAIT_COMM, true, ESTOP_NONE);                /* 初期化OK → 通信待機 */

    in = base(); in.wdt_reboot = true;
    EXPECT(FS_BOOT, in, FS_EMERGENCY_STOP, true, ESTOP_WDT_REBOOT);     /* WDT再起動 → 非常停止 */

    in = base(); in.boot_imu_ok = false;
    EXPECT(FS_BOOT, in, FS_EMERGENCY_STOP, true, ESTOP_IMU_INIT_FAIL);  /* IMU無応答 → 非常停止 */

    in = base(); in.wdt_reboot = true; in.boot_imu_ok = false;
    EXPECT(FS_BOOT, in, FS_EMERGENCY_STOP, true, ESTOP_WDT_REBOOT);     /* 両方なら根本のWDTを記録 */

    in = base(); in.estop_request = true;
    EXPECT(FS_BOOT, in, FS_EMERGENCY_STOP, true, ESTOP_OPERATOR);

    /* BOOT では他の入力は無視（アーム要求などで先へ進まない） */
    in = base(); in.arm_rising_edge = true; in.level_and_zero_throttle = true; in.comm_connected = true;
    in.comm_lost = true; in.battery_low = true; in.imu_timeout = true;
    EXPECT(FS_BOOT, in, FS_WAIT_COMM, true, ESTOP_NONE);
}

static void test_fsm_wait_comm(void)
{
    flight_fsm_inputs_t in = base();
    EXPECT(FS_WAIT_COMM, in, FS_WAIT_COMM, false, ESTOP_NONE);

    in = base(); in.comm_connected = true;
    EXPECT(FS_WAIT_COMM, in, FS_STANDBY, true, ESTOP_NONE);             /* 最初の有効パケット → 待機 */

    in = base(); in.comm_connected = true; in.comm_lost = true;         /* 矛盾した入力は途絶側 */
    EXPECT(FS_WAIT_COMM, in, FS_WAIT_COMM, false, ESTOP_NONE);

    in = base(); in.imu_timeout = true;                                 /* 提案: 停止中でもIMU途絶は非常停止 */
    EXPECT(FS_WAIT_COMM, in, FS_EMERGENCY_STOP, true, ESTOP_IMU_TIMEOUT);

    in = base(); in.estop_request = true; in.comm_connected = true;
    EXPECT(FS_WAIT_COMM, in, FS_EMERGENCY_STOP, true, ESTOP_OPERATOR);

    /* 通信待機ではアーム・ディスアーム・電池・傾き・着地は何も起こさない */
    in = base(); in.arm_rising_edge = true; in.level_and_zero_throttle = true; in.disarm_request = true;
    in.battery_low = true; in.tilt_rad = 90.0f * DEG; in.landed = true; in.descent_timeout = true;
    EXPECT(FS_WAIT_COMM, in, FS_WAIT_COMM, false, ESTOP_NONE);
}

static void test_fsm_standby_arm(void)
{
    flight_fsm_inputs_t in = base();
    EXPECT(FS_STANDBY, in, FS_STANDBY, false, ESTOP_NONE);

    /* アームは「0→1の周期」かつ「水平・スロットル0」の両方が必要 */
    in = base(); in.arm_rising_edge = true;
    EXPECT(FS_STANDBY, in, FS_STANDBY, false, ESTOP_NONE);              /* 傾いている/スロットルあり */
    in = base(); in.level_and_zero_throttle = true;
    EXPECT(FS_STANDBY, in, FS_STANDBY, false, ESTOP_NONE);              /* 旗が立ち続けているだけ（辺なし） */
    in = base(); in.arm_rising_edge = true; in.level_and_zero_throttle = true;
    EXPECT(FS_STANDBY, in, FS_FLIGHT, true, ESTOP_NONE);

    /* ディスアーム要求と同時なら見送る（提案） */
    in.disarm_request = true;
    EXPECT(FS_STANDBY, in, FS_STANDBY, false, ESTOP_NONE);

    /* 通信途絶は優先して通信待機へ（提案 #20）。辺があってもアームしない */
    in = base(); in.arm_rising_edge = true; in.level_and_zero_throttle = true; in.comm_lost = true;
    EXPECT(FS_STANDBY, in, FS_WAIT_COMM, true, ESTOP_NONE);

    in = base(); in.imu_timeout = true;
    EXPECT(FS_STANDBY, in, FS_EMERGENCY_STOP, true, ESTOP_IMU_TIMEOUT);
    in = base(); in.estop_request = true; in.arm_rising_edge = true; in.level_and_zero_throttle = true;
    EXPECT(FS_STANDBY, in, FS_EMERGENCY_STOP, true, ESTOP_OPERATOR);

    /* 待機中は電池・傾き・着地では何も起こさない（モーターは止まっている） */
    in = base(); in.battery_low = true; in.tilt_rad = 90.0f * DEG; in.landed = true; in.descent_timeout = true;
    EXPECT(FS_STANDBY, in, FS_STANDBY, false, ESTOP_NONE);
}

static void test_fsm_flight(void)
{
    flight_fsm_inputs_t in = base();
    EXPECT(FS_FLIGHT, in, FS_FLIGHT, false, ESTOP_NONE);

    in = base(); in.disarm_request = true;
    EXPECT(FS_FLIGHT, in, FS_STANDBY, true, ESTOP_NONE);                /* ディスアーム → 待機 */

    flight_fsm_t f;
    in = base(); in.comm_lost = true;
    run_from(FS_FLIGHT, &in, &f);
    CHECK(f.state == FS_EMERGENCY_DESCENT); CHECK(f.descent_reason == DESCENT_COMM_LOST);
    CHECK(f.estop_reason == ESTOP_NONE);
    in = base(); in.battery_low = true;
    run_from(FS_FLIGHT, &in, &f);
    CHECK(f.state == FS_EMERGENCY_DESCENT); CHECK(f.descent_reason == DESCENT_BATTERY_LOW);
    in = base(); in.battery_low = true; in.comm_lost = true;
    run_from(FS_FLIGHT, &in, &f);
    CHECK(f.descent_reason == DESCENT_COMM_LOST);                       /* 両方なら通信途絶を記録 */

    /* 傾き: ちょうど60°は許容、超えたら非常停止。NaN も非常停止 */
    in = base(); in.tilt_rad = TILT_ESTOP;
    EXPECT(FS_FLIGHT, in, FS_FLIGHT, false, ESTOP_NONE);
    in = base(); in.tilt_rad = TILT_ESTOP + 0.001f;
    EXPECT(FS_FLIGHT, in, FS_EMERGENCY_STOP, true, ESTOP_TILT_EXCEEDED);
    in = base(); in.tilt_rad = NAN;
    EXPECT(FS_FLIGHT, in, FS_EMERGENCY_STOP, true, ESTOP_TILT_EXCEEDED);

    in = base(); in.imu_timeout = true;
    EXPECT(FS_FLIGHT, in, FS_EMERGENCY_STOP, true, ESTOP_IMU_TIMEOUT);
    in = base(); in.estop_request = true;
    EXPECT(FS_FLIGHT, in, FS_EMERGENCY_STOP, true, ESTOP_OPERATOR);

    /* 優先順: 非常停止 > ディスアーム > 降下 */
    in = base(); in.tilt_rad = 1.5f; in.comm_lost = true; in.disarm_request = true;
    EXPECT(FS_FLIGHT, in, FS_EMERGENCY_STOP, true, ESTOP_TILT_EXCEEDED);
    in = base(); in.disarm_request = true; in.comm_lost = true; in.battery_low = true;
    EXPECT(FS_FLIGHT, in, FS_STANDBY, true, ESTOP_NONE);
    in = base(); in.imu_timeout = true; in.tilt_rad = 1.5f;
    EXPECT(FS_FLIGHT, in, FS_EMERGENCY_STOP, true, ESTOP_IMU_TIMEOUT);

    /* 飛行中のアーム辺・着地・時間切れ・接続は無視 */
    in = base(); in.arm_rising_edge = true; in.level_and_zero_throttle = true; in.comm_connected = true;
    in.landed = true; in.descent_timeout = true;
    EXPECT(FS_FLIGHT, in, FS_FLIGHT, false, ESTOP_NONE);
}

static void test_fsm_descent(void)
{
    flight_fsm_inputs_t in = base();
    EXPECT(FS_EMERGENCY_DESCENT, in, FS_EMERGENCY_DESCENT, false, ESTOP_NONE);

    flight_fsm_t f;
    in = base(); in.landed = true;
    run_from(FS_EMERGENCY_DESCENT, &in, &f);
    CHECK(f.state == FS_EMERGENCY_STOP); CHECK(f.estop_reason == ESTOP_DESCENT_LANDED);
    in = base(); in.descent_timeout = true;
    EXPECT(FS_EMERGENCY_DESCENT, in, FS_EMERGENCY_STOP, true, ESTOP_DESCENT_TIMEOUT);
    in = base(); in.descent_timeout = true; in.landed = true;
    EXPECT(FS_EMERGENCY_DESCENT, in, FS_EMERGENCY_STOP, true, ESTOP_DESCENT_LANDED);

    /* 降下の原因は非常停止に入ったあとも残る（ログに両方出す。#5） */
    flight_fsm_init(&f, 0);
    f.state = FS_EMERGENCY_DESCENT; f.descent_reason = DESCENT_BATTERY_LOW;
    in = base(); in.landed = true;
    flight_fsm_step(&f, &in, TILT_ESTOP, 5);
    CHECK(f.state == FS_EMERGENCY_STOP); CHECK(f.descent_reason == DESCENT_BATTERY_LOW);

    /* 降下は戻らない: 通信が戻っても、ディスアーム/アーム辺が来ても降下のまま */
    in = base(); in.comm_connected = true; in.disarm_request = true; in.arm_rising_edge = true;
    in.level_and_zero_throttle = true;
    EXPECT(FS_EMERGENCY_DESCENT, in, FS_EMERGENCY_DESCENT, false, ESTOP_NONE);

    in = base(); in.imu_timeout = true;                                 /* 提案 */
    EXPECT(FS_EMERGENCY_DESCENT, in, FS_EMERGENCY_STOP, true, ESTOP_IMU_TIMEOUT);
    in = base(); in.tilt_rad = 70.0f * DEG;                             /* 提案 */
    EXPECT(FS_EMERGENCY_DESCENT, in, FS_EMERGENCY_STOP, true, ESTOP_TILT_EXCEEDED);
    in = base(); in.estop_request = true;
    EXPECT(FS_EMERGENCY_DESCENT, in, FS_EMERGENCY_STOP, true, ESTOP_OPERATOR);
}

static void test_fsm_estop_latches(void)
{
    /* 非常停止からは、入力が何であっても抜けない・理由も変わらない（乱数で全ビットを振る） */
    uint32_t rng = 777;
    for (int n = 0; n < 5000; n++) {
        flight_fsm_t f;
        flight_fsm_init(&f, 0);
        f.state = FS_EMERGENCY_STOP; f.estop_reason = ESTOP_TILT_EXCEEDED; f.descent_reason = DESCENT_COMM_LOST;
        rng = rng * 1664525u + 1013904223u;
        flight_fsm_inputs_t in;
        in.boot_imu_ok = rng & 1; in.wdt_reboot = (rng >> 1) & 1; in.imu_timeout = (rng >> 2) & 1;
        in.comm_connected = (rng >> 3) & 1; in.comm_lost = (rng >> 4) & 1; in.arm_rising_edge = (rng >> 5) & 1;
        in.level_and_zero_throttle = (rng >> 6) & 1; in.disarm_request = (rng >> 7) & 1;
        in.estop_request = (rng >> 8) & 1; in.battery_low = (rng >> 9) & 1; in.landed = (rng >> 10) & 1;
        in.descent_timeout = (rng >> 11) & 1; in.tilt_rad = ((rng >> 12) & 1) ? 0.0f : 3.0f;
        bool ch = flight_fsm_step(&f, &in, TILT_ESTOP, 100u + (uint32_t)n);
        if (ch || f.state != FS_EMERGENCY_STOP || f.estop_reason != ESTOP_TILT_EXCEEDED ||
            f.descent_reason != DESCENT_COMM_LOST || f.state_enter_us != 0) {
            CHECK(false);   /* 失敗時だけ数える */
            break;
        }
    }
    CHECK(true);
}

static void test_fsm_operator_estop_everywhere(void)
{
    const flight_state_t all[] = { FS_BOOT, FS_WAIT_COMM, FS_STANDBY, FS_FLIGHT, FS_EMERGENCY_DESCENT };
    for (unsigned i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
        flight_fsm_inputs_t in = base(); in.estop_request = true;
        EXPECT(all[i], in, FS_EMERGENCY_STOP, true, ESTOP_OPERATOR);
    }
    /* すでに非常停止なら変わらない（理由も最初のまま） */
    flight_fsm_t f; flight_fsm_init(&f, 0);
    f.state = FS_EMERGENCY_STOP; f.estop_reason = ESTOP_IMU_TIMEOUT;
    flight_fsm_inputs_t in = base(); in.estop_request = true;
    CHECK(!flight_fsm_step(&f, &in, TILT_ESTOP, 9));
    CHECK(f.estop_reason == ESTOP_IMU_TIMEOUT);
}

static void test_fsm_misc(void)
{
    flight_fsm_t f;
    flight_fsm_init(&f, 42);
    CHECK(f.state == FS_BOOT); CHECK(f.estop_reason == ESTOP_NONE);
    CHECK(f.descent_reason == DESCENT_NONE); CHECK(f.state_enter_us == 42);

    /* 遷移したときだけ state_enter_us が更新される */
    flight_fsm_inputs_t in = base();
    flight_fsm_step(&f, &in, TILT_ESTOP, 100);
    CHECK(f.state == FS_WAIT_COMM); CHECK(f.state_enter_us == 100);
    flight_fsm_step(&f, &in, TILT_ESTOP, 200);
    CHECK(f.state_enter_us == 100);

    /* モーターを回してよいのは FLIGHT と DESCENT だけ */
    CHECK(!flight_fsm_motors_allowed(FS_BOOT));
    CHECK(!flight_fsm_motors_allowed(FS_WAIT_COMM));
    CHECK(!flight_fsm_motors_allowed(FS_STANDBY));
    CHECK(flight_fsm_motors_allowed(FS_FLIGHT));
    CHECK(flight_fsm_motors_allowed(FS_EMERGENCY_DESCENT));
    CHECK(!flight_fsm_motors_allowed(FS_EMERGENCY_STOP));

    CHECK(strcmp(flight_state_name(FS_EMERGENCY_STOP), "ESTOP") == 0);
    CHECK(strcmp(estop_reason_name(ESTOP_OPERATOR), "operator") == 0);
    CHECK(strcmp(descent_reason_name(DESCENT_COMM_LOST), "comm_lost") == 0);

    /* 不正な状態値は安全側（非常停止） */
    flight_fsm_init(&f, 0);
    f.state = (flight_state_t)99;
    CHECK(flight_fsm_step(&f, &in, TILT_ESTOP, 1));
    CHECK(f.state == FS_EMERGENCY_STOP); CHECK(f.estop_reason == ESTOP_INTERNAL_ERROR);
}

/** 乱数の入力で長く回し、状態の不変条件を確かめる。 */
static void test_fsm_random_invariants(void)
{
    uint32_t rng = 4242;
    int bad = 0;
    flight_fsm_t f;
    flight_fsm_init(&f, 0);
    for (int n = 0; n < 200000; n++) {
        if (f.state == FS_EMERGENCY_STOP) { flight_fsm_init(&f, 0); }   /* 電源入れ直し相当 */
        flight_state_t prev = f.state;
        flight_fsm_inputs_t in;
        rng = rng * 1664525u + 1013904223u; uint32_t a = rng;
        rng = rng * 1664525u + 1013904223u; uint32_t b = rng;
        /* 非常停止系の入力は低確率にして、状態が先へ進むようにする */
        in.boot_imu_ok = (a & 15) != 0; in.wdt_reboot = (a & 0xff0) == 0; in.imu_timeout = (a & 0xff000) == 0;
        in.comm_connected = (a >> 20) & 1; in.comm_lost = ((a >> 21) & 7) == 0;
        in.arm_rising_edge = (b & 3) == 0; in.level_and_zero_throttle = (b >> 2) & 1;
        in.disarm_request = ((b >> 3) & 15) == 0; in.estop_request = ((b >> 7) & 0x3ff) == 0;
        in.battery_low = ((b >> 17) & 31) == 0; in.landed = ((b >> 22) & 15) == 0;
        in.descent_timeout = ((b >> 26) & 31) == 0; in.tilt_rad = (b >> 31) ? 0.0f : 0.5f;
        flight_fsm_step(&f, &in, TILT_ESTOP, (uint64_t)n);
        flight_state_t now = f.state;
        if (now == FS_FLIGHT && prev != FS_FLIGHT &&
            !(prev == FS_STANDBY && in.arm_rising_edge && in.level_and_zero_throttle && !in.disarm_request)) bad++;
        if (now == FS_EMERGENCY_DESCENT && prev != FS_EMERGENCY_DESCENT && prev != FS_FLIGHT) bad++;
        if (now == FS_EMERGENCY_DESCENT && f.descent_reason == DESCENT_NONE) bad++;
        if (now == FS_EMERGENCY_STOP && f.estop_reason == ESTOP_NONE) bad++;
        if (now != FS_EMERGENCY_STOP && f.estop_reason != ESTOP_NONE) bad++;
        if (now == FS_STANDBY && prev != FS_STANDBY && prev != FS_WAIT_COMM && prev != FS_FLIGHT) bad++;
        if (prev == FS_EMERGENCY_DESCENT && now != FS_EMERGENCY_DESCENT && now != FS_EMERGENCY_STOP) bad++;
        if (flight_fsm_motors_allowed(now) != (now == FS_FLIGHT || now == FS_EMERGENCY_DESCENT)) bad++;
        if (in.estop_request && now != FS_EMERGENCY_STOP) bad++;   /* 操作者の非常停止は即時 */
    }
    CHECK(bad == 0);
}

const test_t g_tests_fsm[] = {
    { "fsm: BOOT の遷移（WDT・IMU・操作）", test_fsm_boot },
    { "fsm: WAIT_COMM の遷移", test_fsm_wait_comm },
    { "fsm: STANDBY とアーム条件（辺・水平・スロットル0）", test_fsm_standby_arm },
    { "fsm: FLIGHT の遷移と優先順", test_fsm_flight },
    { "fsm: EMERGENCY_DESCENT の遷移", test_fsm_descent },
    { "fsm: 非常停止は何があっても抜けない", test_fsm_estop_latches },
    { "fsm: 操作者の非常停止はどの状態からでも", test_fsm_operator_estop_everywhere },
    { "fsm: 初期値・時刻・名前・モーター許可", test_fsm_misc },
    { "fsm: 乱数の入力で不変条件", test_fsm_random_invariants },
};
const int g_tests_fsm_n = (int)(sizeof(g_tests_fsm) / sizeof(g_tests_fsm[0]));
