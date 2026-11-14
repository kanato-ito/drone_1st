/**
 * @file test_guard_descent.c
 * @brief guard（PIDの見張り役）と emergency_descent（降下の目標・着地検出）の試験。どちらも純粋関数。
 */
#include "guard.h"
#include "emergency_descent.h"
#include "params.h"
#include "test_common.h"

static params_t GP;

static attitude_state_t level_att(void)
{
    attitude_state_t a;
    memset(&a, 0, sizeof(a));
    a.valid = true;
    a.accel_g.z = 1.0f;
    return a;
}

static controller_output_t out_ok(void)
{
    controller_output_t o = { 10.0f, { 0.5f, -0.5f, 0.1f } };
    return o;
}

/* ------------------------------------------------------------ guard */

static void test_guard_pre_conditions(void)
{
    params_set_defaults(&GP);
    const guard_params_t *gp = &GP.guard;
    guard_t g;
    attitude_state_t a = level_att();

    guard_reset(&g);
    CHECK(!guard_taken_over(&g));
    CHECK(!guard_check_pre(&g, gp, &a, false));                 /* 水平・静か・人の指示なし: 取り上げない */
    CHECK(g.reason == TAKEOVER_NONE);

    /* 傾き: しきい値ちょうどは許容、超えたら TILT */
    a.angle.roll_rad = gp->tilt_takeover_rad - 0.001f;
    CHECK(!guard_check_pre(&g, gp, &a, false));
    a.angle.roll_rad = gp->tilt_takeover_rad + 0.01f;
    CHECK(guard_check_pre(&g, gp, &a, false));
    CHECK(g.reason == TAKEOVER_TILT);

    /* ピッチだけでも、ロール・ピッチ合成でも */
    guard_reset(&g); a = level_att(); a.angle.pitch_rad = -gp->tilt_takeover_rad - 0.01f;
    CHECK(guard_check_pre(&g, gp, &a, false)); CHECK(g.reason == TAKEOVER_TILT);
    guard_reset(&g); a = level_att();
    a.angle.roll_rad = 0.8f * gp->tilt_takeover_rad; a.angle.pitch_rad = 0.8f * gp->tilt_takeover_rad;
    CHECK(guard_check_pre(&g, gp, &a, false)); CHECK(g.reason == TAKEOVER_TILT);

    /* 角速度: 大きさで判定（軸1つでも、合成でも） */
    guard_reset(&g); a = level_att(); a.rate_rps.x = gp->rate_takeover_rps - 0.1f;
    CHECK(!guard_check_pre(&g, gp, &a, false));
    a.rate_rps.x = gp->rate_takeover_rps + 0.1f;
    CHECK(guard_check_pre(&g, gp, &a, false)); CHECK(g.reason == TAKEOVER_RATE);
    guard_reset(&g); a = level_att(); a.rate_rps.y = 5.0f; a.rate_rps.z = 5.0f;   /* 合成 7.07 → まだ未満 */
    CHECK(!guard_check_pre(&g, gp, &a, false));
    a.rate_rps.x = 5.0f;                                                            /* 合成 8.66 */
    CHECK(guard_check_pre(&g, gp, &a, false)); CHECK(g.reason == TAKEOVER_RATE);

    /* 人の指示 */
    guard_reset(&g); a = level_att();
    CHECK(guard_check_pre(&g, gp, &a, true)); CHECK(g.reason == TAKEOVER_HUMAN);

    /* NaN の姿勢・角速度は異常とみなす */
    guard_reset(&g); a = level_att(); a.angle.roll_rad = NAN;
    CHECK(guard_check_pre(&g, gp, &a, false)); CHECK(g.reason == TAKEOVER_TILT);
    guard_reset(&g); a = level_att(); a.rate_rps.z = NAN;
    CHECK(guard_check_pre(&g, gp, &a, false)); CHECK(g.reason == TAKEOVER_RATE);
}

static void test_guard_post_conditions(void)
{
    params_set_defaults(&GP);
    const guard_params_t *gp = &GP.guard;
    guard_t g;
    controller_output_t o = out_ok();
    const float tmax = 39.0f, qmax = 6.0f;

    guard_reset(&g);
    CHECK(!guard_check_post(&g, gp, &o, tmax, qmax, 200));
    CHECK(g.reason == TAKEOVER_NONE);

    /* NaN / Inf */
    controller_output_t bad = o; bad.torque_g[1] = NAN;
    guard_reset(&g); CHECK(guard_check_post(&g, gp, &bad, tmax, qmax, 200)); CHECK(g.reason == TAKEOVER_POLICY_NAN);
    bad = o; bad.thrust_g = INFINITY;
    guard_reset(&g); CHECK(guard_check_post(&g, gp, &bad, tmax, qmax, 200)); CHECK(g.reason == TAKEOVER_POLICY_NAN);
    bad = o; bad.torque_g[2] = -INFINITY;
    guard_reset(&g); CHECK(guard_check_post(&g, gp, &bad, tmax, qmax, 200)); CHECK(g.reason == TAKEOVER_POLICY_NAN);

    /* 計算時間: 予算ちょうどは許容、超えたら OVERRUN */
    guard_reset(&g); CHECK(!guard_check_post(&g, gp, &o, tmax, qmax, gp->policy_budget_us));
    CHECK(guard_check_post(&g, gp, &o, tmax, qmax, gp->policy_budget_us + 1));
    CHECK(g.reason == TAKEOVER_POLICY_OVERRUN);

    /* 張りつき: policy_sat_cycles 周期続いたら。1周期でも外れたら数え直し */
    controller_output_t sat = o; sat.torque_g[0] = qmax;           /* トルクが上限 */
    guard_reset(&g);
    for (unsigned i = 0; i + 1 < gp->policy_sat_cycles; i++) CHECK(!guard_check_post(&g, gp, &sat, tmax, qmax, 100));
    CHECK(g.sat_count == gp->policy_sat_cycles - 1u);
    CHECK(!guard_check_post(&g, gp, &o, tmax, qmax, 100));         /* 外れる */
    CHECK(g.sat_count == 0);
    for (unsigned i = 0; i + 1 < gp->policy_sat_cycles; i++) CHECK(!guard_check_post(&g, gp, &sat, tmax, qmax, 100));
    CHECK(guard_check_post(&g, gp, &sat, tmax, qmax, 100));        /* ちょうど N 周期目 */
    CHECK(g.reason == TAKEOVER_POLICY_SATURATED);

    /* 推力の張りつき・負側も */
    controller_output_t sat_t = o; sat_t.thrust_g = tmax;
    guard_reset(&g);
    for (unsigned i = 0; i < gp->policy_sat_cycles; i++) guard_check_post(&g, gp, &sat_t, tmax, qmax, 100);
    CHECK(g.reason == TAKEOVER_POLICY_SATURATED);
    controller_output_t sat_n = o; sat_n.torque_g[2] = -qmax;
    guard_reset(&g);
    for (unsigned i = 0; i < gp->policy_sat_cycles; i++) guard_check_post(&g, gp, &sat_n, tmax, qmax, 100);
    CHECK(g.reason == TAKEOVER_POLICY_SATURATED);

    /* 上限の ratio 倍未満なら張りつきではない */
    controller_output_t near = o; near.torque_g[0] = qmax * (gp->policy_sat_ratio - 0.02f);
    guard_reset(&g);
    for (unsigned i = 0; i < 3u * gp->policy_sat_cycles; i++) guard_check_post(&g, gp, &near, tmax, qmax, 100);
    CHECK(g.reason == TAKEOVER_NONE);
}

static void test_guard_latch_and_reset(void)
{
    params_set_defaults(&GP);
    const guard_params_t *gp = &GP.guard;
    guard_t g;
    attitude_state_t a = level_att();
    controller_output_t o = out_ok();

    guard_reset(&g);
    a.angle.roll_rad = 0.9f;                                       /* 傾いて取り上げ */
    CHECK(guard_check_pre(&g, gp, &a, false));
    CHECK(g.reason == TAKEOVER_TILT);

    /* 水平に戻っても、出力が正常でも、取り上げたまま（ラッチ）。理由は最初のまま */
    a = level_att();
    CHECK(guard_check_pre(&g, gp, &a, false));
    CHECK(guard_check_post(&g, gp, &o, 39.0f, 6.0f, 100));
    CHECK(guard_taken_over(&g)); CHECK(g.reason == TAKEOVER_TILT);
    controller_output_t bad = o; bad.thrust_g = NAN;
    CHECK(guard_check_post(&g, gp, &bad, 39.0f, 6.0f, 100));
    CHECK(g.reason == TAKEOVER_TILT);                              /* 後の異常で理由を上書きしない */
    guard_force_takeover(&g, TAKEOVER_POLICY_FAIL);
    CHECK(g.reason == TAKEOVER_TILT);

    /* 解除は guard_reset だけ */
    guard_reset(&g);
    CHECK(!guard_taken_over(&g)); CHECK(g.sat_count == 0);
    CHECK(!guard_check_pre(&g, gp, &a, false));

    /* 強制の取り上げ（方策の step 失敗） */
    guard_force_takeover(&g, TAKEOVER_POLICY_FAIL);
    CHECK(guard_taken_over(&g)); CHECK(g.reason == TAKEOVER_POLICY_FAIL);
    CHECK(strcmp(takeover_reason_name(TAKEOVER_POLICY_FAIL), "policy_fail") == 0);
}

/* ------------------------------------------------------------ descent */

#define MS(x) ((uint64_t)(x) * 1000u)

static void test_descent_target_thrust(void)
{
    params_set_defaults(&GP);
    const descent_params_t *dp = &GP.descent;
    descent_t d; descent_init(&d);
    attitude_state_t a = level_att();
    a.angle.yaw_rad = 1.2f;
    descent_target_t t;

    /* ホバリング推力の推定: 最初の1回はそのまま、あとはゆっくり追う */
    descent_track_hover(&d, 8.0f);
    CHECK_NEAR(d.hover_thrust_est_g, 8.0f, 1e-6f);
    for (int i = 0; i < 5000; i++) descent_track_hover(&d, 10.0f);
    CHECK_NEAR(d.hover_thrust_est_g, 10.0f, 0.01f);                /* 約0.5秒の時定数で収束 */
    descent_track_hover(&d, NAN);                                  /* 異常値は取り込まない */
    descent_track_hover(&d, -1.0f);
    CHECK_NEAR(d.hover_thrust_est_g, 10.0f, 0.01f);

    descent_begin(&d, &a, MS(1000));
    descent_update(&d, dp, &a, MS(1001), &t);
    CHECK_NEAR(t.target_thrust_g, dp->thrust_ratio * d.hover_thrust_est_g, 1e-5f);   /* 比 × 推定 */
    CHECK_NEAR(t.target_thrust_g, 9.0f, 0.02f);
    CHECK(t.target_attitude.roll_rad == 0.0f); CHECK(t.target_attitude.pitch_rad == 0.0f);
    CHECK_NEAR(t.target_attitude.yaw_rad, 1.2f, 1e-6f);            /* ヨーは降下開始時の値を保持 */
    CHECK(!t.landed); CHECK(!t.timed_out);

    /* 降下中にヨーが動いても目標は開始時のまま。降下中はホバリング推定を更新しなくても目標は同じ */
    a.angle.yaw_rad = -2.0f;
    descent_update(&d, dp, &a, MS(1500), &t);
    CHECK_NEAR(t.target_attitude.yaw_rad, 1.2f, 1e-6f);
    CHECK_NEAR(t.target_thrust_g, 9.0f, 0.02f);

    /* ratio を変えると追従 */
    descent_params_t dp2 = *dp; dp2.thrust_ratio = 0.5f;
    descent_update(&d, &dp2, &a, MS(1500), &t);
    CHECK_NEAR(t.target_thrust_g, 5.0f, 0.02f);

    /* 飛んでいない（推定0）なら目標推力0 */
    descent_t d0; descent_init(&d0);
    descent_begin(&d0, &a, 0);
    descent_update(&d0, dp, &a, 10, &t);
    CHECK(t.target_thrust_g == 0.0f);
}

static void test_descent_landing_detection(void)
{
    params_set_defaults(&GP);
    descent_params_t dp = GP.descent;      /* shock 3g, quiet rate<0.3, tol 0.1g, hold 500ms, max 3000ms */
    descent_t d; descent_init(&d); descent_track_hover(&d, 8.0f);
    attitude_state_t a = level_att();
    descent_target_t t;
    uint64_t now = MS(10000);
    descent_begin(&d, &a, now);

    /* (1) 衝撃なしで静かな状態が続いても着地とは言わない（空中でゆっくり降りているだけかもしれない） */
    for (int i = 0; i < 400; i++) { now += MS(1); descent_update(&d, &dp, &a, now, &t); }
    CHECK(!t.landed);
    for (int i = 0; i < 800; i++) { now += MS(1); descent_update(&d, &dp, &a, now, &t); }   /* 合計 1.2s > hold */
    CHECK(!t.landed);

    /* (2) 衝撃のあと静かな状態が hold の間続くと着地 */
    a.accel_g.z = 4.0f;                                            /* 衝撃 */
    now += MS(1); descent_update(&d, &dp, &a, now, &t);
    CHECK(!t.landed); CHECK(d.shock_seen);
    a.accel_g.z = 1.0f;                                            /* 静か */
    now += MS(1); descent_update(&d, &dp, &a, now, &t);            /* 静かの開始 */
    uint64_t quiet_start = now;
    while (now - quiet_start < MS(499)) { now += MS(1); descent_update(&d, &dp, &a, now, &t); CHECK(!t.landed); }
    now += MS(1); descent_update(&d, &dp, &a, now, &t);            /* ちょうど 500ms */
    CHECK(t.landed);

    /* (3) 動くと静かな時間は数え直し: 400ms 静か → 動き(角速度) → 400ms 静か → まだ。その後 500ms で着地 */
    descent_init(&d); descent_track_hover(&d, 8.0f); a = level_att();
    now = MS(20000); descent_begin(&d, &a, now);
    a.accel_g.z = 3.5f; now += MS(1); descent_update(&d, &dp, &a, now, &t);
    a.accel_g.z = 1.0f;
    for (int i = 0; i < 400; i++) { now += MS(1); descent_update(&d, &dp, &a, now, &t); }
    CHECK(!t.landed);
    a.rate_rps.x = 1.0f;                                           /* 動いた */
    now += MS(1); descent_update(&d, &dp, &a, now, &t);
    CHECK(!d.quiet_active);
    a.rate_rps.x = 0.0f;
    for (int i = 0; i < 400; i++) { now += MS(1); descent_update(&d, &dp, &a, now, &t); }
    CHECK(!t.landed);                                              /* 通算では 800ms だが、数え直したので 400ms */
    for (int i = 0; i < 101; i++) { now += MS(1); descent_update(&d, &dp, &a, now, &t); }
    CHECK(t.landed);

    /* 加速度が 1g からずれる（浮いている・傾いて引きずられる）場合も動きとして数え直す */
    descent_init(&d); descent_track_hover(&d, 8.0f); a = level_att();
    now = MS(30000); descent_begin(&d, &a, now);
    a.accel_g.z = 3.5f; now += MS(1); descent_update(&d, &dp, &a, now, &t);
    a.accel_g.z = 1.0f;
    for (int i = 0; i < 400; i++) { now += MS(1); descent_update(&d, &dp, &a, now, &t); }
    a.accel_g.z = 1.0f + dp.quiet_accel_tol_g + 0.05f;
    now += MS(1); descent_update(&d, &dp, &a, now, &t);
    CHECK(!d.quiet_active);
    a.accel_g.z = 1.0f;
    for (int i = 0; i < 400; i++) { now += MS(1); descent_update(&d, &dp, &a, now, &t); }
    CHECK(!t.landed);

    /* 後からもう一度衝撃（バウンド）が来ても、その後の静かさで着地 */
    a.accel_g.z = 3.5f; now += MS(1); descent_update(&d, &dp, &a, now, &t);
    CHECK(!d.quiet_active);
    a.accel_g.z = 1.0f;
    for (int i = 0; i < 520; i++) { now += MS(1); descent_update(&d, &dp, &a, now, &t); }
    CHECK(t.landed);

    /* しきい値ちょうどの衝撃(= shock_g)はまだ衝撃ではない */
    descent_init(&d); a = level_att();
    descent_begin(&d, &a, 0);
    a.accel_g.z = dp.shock_g;
    descent_update(&d, &dp, &a, MS(1), &t);
    CHECK(!d.shock_seen);
}

static void test_descent_timeout(void)
{
    params_set_defaults(&GP);
    const descent_params_t *dp = &GP.descent;
    descent_t d; descent_init(&d); descent_track_hover(&d, 8.0f);
    attitude_state_t a = level_att();
    descent_target_t t;
    descent_begin(&d, &a, MS(5000));
    descent_update(&d, dp, &a, MS(5000) + MS(dp->max_time_ms) - 1u, &t);
    CHECK(!t.timed_out);
    descent_update(&d, dp, &a, MS(5000) + MS(dp->max_time_ms), &t);
    CHECK(t.timed_out);                                            /* 衝撃が無くても時間切れで止まる */
    CHECK(!t.landed);

    /* 時刻が64bitで大きくてもよい */
    descent_begin(&d, &a, (uint64_t)1 << 40);
    descent_update(&d, dp, &a, ((uint64_t)1 << 40) + MS(100), &t);
    CHECK(!t.timed_out);
}

static void test_params_safety_validate(void)
{
    params_t p;
    params_set_defaults(&p);
    CHECK(params_validate(&p));
    p.safety.wdt_timeout_ms = 10;        CHECK(!params_validate(&p)); params_set_defaults(&p);
    p.safety.comm_timeout_ms = 0;        CHECK(!params_validate(&p)); params_set_defaults(&p);
    p.safety.tilt_estop_rad = 0.0f;      CHECK(!params_validate(&p)); params_set_defaults(&p);
    p.guard.tilt_takeover_rad = p.safety.tilt_estop_rad; CHECK(!params_validate(&p)); params_set_defaults(&p);
    p.guard.policy_sat_ratio = 1.5f;     CHECK(!params_validate(&p)); params_set_defaults(&p);
    p.descent.thrust_ratio = 1.2f;       CHECK(!params_validate(&p)); params_set_defaults(&p);
    p.descent.max_time_ms = 0;           CHECK(!params_validate(&p)); params_set_defaults(&p);
    p.descent.shock_g = NAN;             CHECK(!params_validate(&p)); params_set_defaults(&p);
    CHECK(params_validate(&p));
}

const test_t g_tests_guard[] = {
    { "params: 安全系しきい値の検査", test_params_safety_validate },
    { "guard: 計算前の判定（傾き・角速度・人の指示・NaN）", test_guard_pre_conditions },
    { "guard: 計算後の判定（NaN・時間超過・張りつき）", test_guard_post_conditions },
    { "guard: ラッチと reset", test_guard_latch_and_reset },
};
const int g_tests_guard_n = (int)(sizeof(g_tests_guard) / sizeof(g_tests_guard[0]));

const test_t g_tests_descent[] = {
    { "descent: 目標推力＝比×ホバリング推定、ヨー保持", test_descent_target_thrust },
    { "descent: 着地検出（衝撃→静かさ保持、動きで数え直し）", test_descent_landing_detection },
    { "descent: 時間切れ", test_descent_timeout },
};
const int g_tests_descent_n = (int)(sizeof(g_tests_descent) / sizeof(g_tests_descent[0]));
