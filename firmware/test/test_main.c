/**
 * @file test_main.c
 * @brief ステップ③a の単体試験（ホストPC）。簡単な自前の check マクロだけで書いてある。
 *
 * 数値の許容幅は「何を確かめたいか」から決めた（各 CHECK の近くに理由を書いた）。
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "params.h"
#include "attitude.h"
#include "controller.h"
#include "controller_pid.h"
#include "mixer.h"
#include "imu_mount.h"
#include "thrust_model.h"
#include "battery.h"
#include "cycle_timer.h"
#include "fake_port.h"
#include "test_common.h"

#define PI_F  3.14159265f

int g_checks, g_fail, g_test_checks;

/* 決定的な乱数（試験を毎回同じ結果にする） */
static uint32_t s_rng = 12345u;
static float frand(void)   /* -1..1 */
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return (float)((s_rng >> 8) & 0xFFFF) / 32768.0f - 1.0f;
}

/* ---------- 姿勢推定の試験の道具 ---------- */
#define SAMPLE_DT (1.0f / 7519.0f)

static void make_batch(imu_batch_t *b, vec3f_t gyro, vec3f_t accel, float gyro_noise)
{
    b->count = 8; b->words_read = 16; b->fifo_overrun = false; b->truncated = false;
    for (int i = 0; i < 8; i++) {
        b->samples[i].gyro_rps.x = gyro.x + gyro_noise * frand();
        b->samples[i].gyro_rps.y = gyro.y + gyro_noise * frand();
        b->samples[i].gyro_rps.z = gyro.z + gyro_noise * frand();
        b->samples[i].accel_g = accel;
    }
}

static void run_batches(attitude_estimator_t *e, int n, vec3f_t gyro, vec3f_t accel, float noise)
{
    static imu_batch_t b;
    for (int i = 0; i < n; i++) {
        make_batch(&b, gyro, accel, noise);
        attitude_update_batch(e, &b, SAMPLE_DT);
    }
}

#define BATCHES_PER_S 940   /* 7519/8 */

static params_t P;

static void test_attitude_still_and_bias(void)
{
    attitude_estimator_t e;
    attitude_init(&e, &P.attitude);
    vec3f_t bias = { 0.010f, -0.020f, 0.005f };
    vec3f_t up = { 0, 0, 1 };

    run_batches(&e, BATCHES_PER_S / 2, bias, up, 0.005f);
    CHECK(!attitude_is_still(&e));          /* 0.5秒ではまだ still_hold_ms(1000)に足りない */
    run_batches(&e, BATCHES_PER_S * 3, bias, up, 0.005f);
    CHECK(attitude_is_still(&e));
    CHECK(attitude_bias_ready(&e));
    /* バイアスが収束: 雑音(±0.005, 8組平均)より十分小さい誤差 */
    CHECK_NEAR(e.bias_rps.x, bias.x, 0.001f);
    CHECK_NEAR(e.bias_rps.y, bias.y, 0.001f);
    CHECK_NEAR(e.bias_rps.z, bias.z, 0.001f);
    /* 静止で姿勢≈0（0.3°未満） */
    CHECK_NEAR(e.out.angle.roll_rad, 0.0f, 0.3f * DEG);
    CHECK_NEAR(e.out.angle.pitch_rad, 0.0f, 0.3f * DEG);

    /* アームで固定。その後ヨーはバイアスを引いて積分されるのでほとんど動かない */
    attitude_set_bias_locked(&e, true);
    vec3f_t bias_after = { 0.030f, 0.0f, 0.020f };  /* 固定中に値が変わっても無視されるはず */
    run_batches(&e, BATCHES_PER_S * 5, bias_after, up, 0.0f);
    CHECK_NEAR(e.bias_rps.x, bias.x, 0.001f);          /* 固定されている */
}

static void test_attitude_not_still_when_moving(void)
{
    attitude_estimator_t e;
    attitude_init(&e, &P.attitude);
    vec3f_t up = { 0, 0, 1 };
    /* ジャイロが毎周期大きく揺れる（手で持って動かしている想定）: ばらつき大 → 静止にならない・バイアスを求めない */
    static imu_batch_t b;
    for (int i = 0; i < BATCHES_PER_S * 4; i++) {
        vec3f_t g = { (i % 2) ? 0.3f : -0.3f, 0, 0 };
        make_batch(&b, g, up, 0.0f);
        attitude_update_batch(&e, &b, SAMPLE_DT);
    }
    CHECK(!attitude_is_still(&e));
    CHECK(!attitude_bias_ready(&e));
    /* 加速度の大きさが1gでない（振っている）ときも静止にしない */
    attitude_init(&e, &P.attitude);
    vec3f_t a15 = { 0, 0, 1.4f };
    run_batches(&e, BATCHES_PER_S * 4, (vec3f_t){0, 0, 0}, a15, 0.0f);
    CHECK(!attitude_is_still(&e));
    /* 一定の速さ(0.5rad/s)で回り続ける: ばらつきは0でも静止にしない・バイアスにしない */
    attitude_init(&e, &P.attitude);
    run_batches(&e, BATCHES_PER_S * 4, (vec3f_t){0.5f, 0, 0}, up, 0.0f);
    CHECK(!attitude_is_still(&e));
    CHECK(!attitude_bias_ready(&e));
}

static void test_attitude_tilt_follows_tau(void)
{
    attitude_estimator_t e;
    attitude_init(&e, &P.attitude);
    vec3f_t zero = {0, 0, 0}, up = {0, 0, 1};
    run_batches(&e, BATCHES_PER_S * 2, zero, up, 0.0f);

    /* 急に30°傾けた状態（ジャイロは0＝回転は見えず、加速度だけが傾きを示す）。
     * 相補フィルタなので角度は時定数τの一次遅れで加速度の値へ寄る */
    float a30 = 30.0f * DEG;
    vec3f_t tilted = { 0, sinf(a30), cosf(a30) };
    float tau = P.attitude.tau_s;
    int n_tau = (int)(tau * (float)BATCHES_PER_S);
    run_batches(&e, n_tau, zero, tilted, 0.0f);
    /* t=τ で 63.2%（30° × 0.632 = 18.96°）。離散化の誤差を見て ±1.5° */
    CHECK_NEAR(e.out.angle.roll_rad / DEG, 30.0f * 0.632f, 1.5f);
    CHECK_NEAR(e.out.angle.pitch_rad / DEG, 0.0f, 0.2f);
    run_batches(&e, n_tau * 5, zero, tilted, 0.0f);
    CHECK_NEAR(e.out.angle.roll_rad / DEG, 30.0f, 0.5f);

    /* ピッチ側（加速度 x が -sinθ）と符号 */
    float p20 = 20.0f * DEG;
    vec3f_t pit = { -sinf(p20), 0, cosf(p20) };
    attitude_init(&e, &P.attitude);
    run_batches(&e, n_tau * 6, zero, pit, 0.0f);
    run_batches(&e, 1, zero, up, 0.0f);
    CHECK_NEAR(e.out.angle.pitch_rad / DEG, 20.0f, 0.5f);
    /* 傾きの合成値 */
    attitude_state_t s = e.out;
    CHECK_NEAR(attitude_tilt_rad(&s) / DEG, 20.0f, 0.6f);
}

static void test_attitude_gyro_tracks_rotation(void)
{
    /* 実際に回転しているとき（ジャイロも加速度も整合）に角度が追従する */
    attitude_estimator_t e;
    attitude_init(&e, &P.attitude);
    vec3f_t zero = {0, 0, 0}, up = {0, 0, 1};
    run_batches(&e, BATCHES_PER_S * 2, zero, up, 0.0f);
    float rate = 1.0f;   /* rad/s でロール */
    for (int i = 0; i < BATCHES_PER_S / 2; i++) {
        float ang = rate * (float)(i + 1) / (float)BATCHES_PER_S;
        vec3f_t g = { rate, 0, 0 }, a = { 0, sinf(ang), cosf(ang) };
        run_batches(&e, 1, g, a, 0.0f);
    }
    CHECK_NEAR(e.out.angle.roll_rad, 0.5f, 0.02f);
    CHECK_NEAR(e.out.rate_rps.x, 1.0f, 0.01f);
}

static void test_attitude_yaw_integrates_and_wraps(void)
{
    attitude_estimator_t e;
    attitude_init(&e, &P.attitude);
    vec3f_t up = {0, 0, 1};
    vec3f_t gz1 = {0, 0, 1.0f};
    /* 水平のまま 1rad/s で1秒 → ヨー1rad（磁気なし＝ジャイロ積分だけ） */
    run_batches(&e, BATCHES_PER_S, gz1, up, 0.0f);
    CHECK_NEAR(e.out.angle.yaw_rad, 1.0f, 0.02f);
    /* さらに3秒 → 4rad は折り返して 4-2π = -2.283 */
    run_batches(&e, BATCHES_PER_S * 3, gz1, up, 0.0f);
    CHECK_NEAR(e.out.angle.yaw_rad, 4.0f - 2.0f * PI_F, 0.05f);
    CHECK(e.out.angle.yaw_rad <= PI_F && e.out.angle.yaw_rad > -PI_F);
}

static void test_wrap_and_input_make(void)
{
    CHECK_NEAR(attitude_wrap_pi(190.0f * DEG) / DEG, -170.0f, 0.01f);
    CHECK_NEAR(attitude_wrap_pi(-190.0f * DEG) / DEG, 170.0f, 0.01f);
    CHECK_NEAR(attitude_wrap_pi(10.0f * DEG) / DEG, 10.0f, 0.001f);
    CHECK_NEAR(attitude_wrap_pi(720.0f * DEG + 30.0f * DEG) / DEG, 30.0f, 0.01f);
    CHECK_NEAR(attitude_wrap_pi(NAN), 0.0f, 0.0f);

    /* ヨー: 機体 170°、目標 -170° → ずれは 340° ではなく -20° */
    attitude_state_t att; memset(&att, 0, sizeof(att));
    att.angle.yaw_rad = 170.0f * DEG;
    att.angle.roll_rad = 0.1f;
    att.rate_rps.x = 0.2f; att.rate_rps.y = 0.3f; att.rate_rps.z = 0.4f;
    euler_t tgt = { 0.05f, -0.06f, -170.0f * DEG };
    controller_output_t prev = { 7.0f, { 1.0f, 2.0f, 3.0f } };
    controller_input_t in;
    controller_input_make(&att, &tgt, 12.0f, &prev, &in);
    CHECK_NEAR(in.attitude.yaw_rad / DEG, -20.0f, 0.01f);
    CHECK_NEAR(in.target_attitude.yaw_rad, 0.0f, 0.0f);

    /* 14要素の並び（学習側と合わせる順序） */
    float v[FC_CONTROLLER_INPUT_DIM];
    controller_input_pack(&in, v);
    CHECK_NEAR(v[0], 0.1f, 1e-6f);  CHECK_NEAR(v[2] / DEG, -20.0f, 0.01f);
    CHECK_NEAR(v[3], 0.2f, 1e-6f);  CHECK_NEAR(v[5], 0.4f, 1e-6f);
    CHECK_NEAR(v[6], 0.05f, 1e-6f); CHECK_NEAR(v[7], -0.06f, 1e-6f);
    CHECK_NEAR(v[9], 12.0f, 1e-6f); CHECK_NEAR(v[10], 7.0f, 1e-6f);
    CHECK_NEAR(v[11], 1.0f, 1e-6f); CHECK_NEAR(v[13], 3.0f, 1e-6f);
}

/* ---------- PID ---------- */
static controller_input_t level_input(float thrust_g)
{
    controller_input_t in; memset(&in, 0, sizeof(in));
    in.target_thrust_g = thrust_g;
    return in;
}

static void test_pid_axis(void)
{
    pid_gains_t g = { 2.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f };
    pid_axis_state_t st; memset(&st, 0, sizeof(st));
    CHECK_NEAR(pid_axis_update(&st, &g, 0.5f, 0.0f, 0.001f, true), 1.0f, 1e-6f);   /* Pのみ */
    g.out_limit = 0.8f;
    CHECK_NEAR(pid_axis_update(&st, &g, 0.5f, 0.0f, 0.001f, true), 0.8f, 1e-6f);   /* 出力上限 */
    CHECK_NEAR(pid_axis_update(&st, &g, -0.5f, 0.0f, 0.001f, true), -0.8f, 1e-6f);
}

static void test_pid_d_wraps_at_180(void)
{
    /* D項だけ。測定値が +179° → -179° と 2° だけ回ったとき、D は 2°ぶんの変化として出る（360°弱の跳ねにならない） */
    pid_gains_t g = { 0.0f, 0.0f, 1.0f, 0.0f, 1000.0f, 0.0f };
    pid_axis_state_t st; memset(&st, 0, sizeof(st));
    const float deg = 3.14159265f / 180.0f, dt = 0.001f;
    (void)pid_axis_update(&st, &g, 0.0f, 179.0f * deg, dt, false);
    float d = pid_axis_update(&st, &g, 0.0f, -179.0f * deg, dt, false);
    CHECK_NEAR(d, -(2.0f * deg) / dt, 0.05f);
}

static void test_pid_integral_conditions(void)
{
    pid_controller_t c;
    controller_pid_init(&c, &P.pid);
    controller_t h = controller_pid_handle(&c);
    controller_output_t out;
    float dt = 1.0f / 940.0f;

    /* スロットルがしきい値以下 → 積分は0のまま（出力は P 項だけ） */
    controller_input_t in = level_input(P.pid.i_enable_thrust_g - 1.0f);
    in.rate_rps.x = -0.5f;                 /* 角速度の誤差が +0.5 の状態を続ける */
    for (int i = 0; i < 2000; i++) h.ops->step(h.self, &in, dt, &out);
    CHECK_NEAR(c.rate[0].integ, 0.0f, 0.0f);
    CHECK_NEAR(out.torque_g[0], P.pid.rate[0].kp * 0.5f, 1e-5f);

    /* しきい値を超えると積分がたまり、上限で止まる */
    in = level_input(P.pid.i_enable_thrust_g + 5.0f);
    in.rate_rps.x = -0.5f;
    h.ops->step(h.self, &in, dt, &out);
    CHECK(c.rate[0].integ > 0.0f);
    for (int i = 0; i < 20000; i++) h.ops->step(h.self, &in, dt, &out);
    CHECK_NEAR(c.rate[0].integ, P.pid.rate[0].i_limit, 1e-6f);   /* 上限で止まっている */
    CHECK_NEAR(out.torque_g[0], P.pid.rate[0].kp * 0.5f + P.pid.rate[0].i_limit, 1e-5f);
    CHECK_NEAR(out.thrust_g, in.target_thrust_g, 0.0f);           /* 推力は素通し */

    /* しきい値以下に戻ると保持（減らさない・増やさない） */
    float held = c.rate[0].integ;
    in.target_thrust_g = 0.0f;
    for (int i = 0; i < 100; i++) h.ops->step(h.self, &in, dt, &out);
    CHECK_NEAR(c.rate[0].integ, held, 0.0f);

    /* reset（アーム時）で0 */
    h.ops->reset(h.self);
    CHECK_NEAR(c.rate[0].integ, 0.0f, 0.0f);
    CHECK_NEAR(c.rate[1].integ, 0.0f, 0.0f);
    CHECK_NEAR(c.rate[2].integ, 0.0f, 0.0f);
}

static void test_pid_cascade_signs(void)
{
    pid_controller_t c;
    controller_pid_init(&c, &P.pid);
    controller_t h = controller_pid_handle(&c);
    controller_output_t out;
    float dt = 1.0f / 940.0f;

    /* ロールが +0.1rad にずれて目標0 → 外側が負の目標角速度 → 内側が負の指令（戻す向き） */
    controller_input_t in = level_input(0.0f);
    in.attitude.roll_rad = 0.1f;
    h.ops->step(h.self, &in, dt, &out);
    CHECK_NEAR(c.last_rate_target_rps[0], -P.pid.angle[0].kp * 0.1f, 1e-5f);
    CHECK_NEAR(out.torque_g[0], P.pid.rate[0].kp * (-P.pid.angle[0].kp * 0.1f), 1e-5f);
    CHECK(out.torque_g[1] == 0.0f && out.torque_g[2] == 0.0f);   /* 他の軸は動かない */

    /* ピッチ・ヨーも同様 */
    in = level_input(0.0f);
    in.attitude.pitch_rad = -0.1f;
    in.attitude.yaw_rad = 0.2f;
    h.ops->step(h.self, &in, dt, &out);
    CHECK(out.torque_g[1] > 0.0f);
    CHECK(out.torque_g[2] < 0.0f);

    /* 出力上限: 巨大な誤差でも out_limit */
    in = level_input(0.0f);
    in.rate_rps.x = 100.0f;
    h.ops->step(h.self, &in, dt, &out);
    CHECK_NEAR(out.torque_g[0], -P.pid.rate[0].out_limit, 1e-5f);
}

/* ---------- ミキサー ---------- */
#define MAXG 30.0f

static controller_output_t U(float t, float r, float p, float y)
{
    controller_output_t u = { t, { r, p, y } };
    return u;
}

static void test_mixer_sign_table(void)
{
    CHECK(mixer_sign_table_valid(P.airframe.mixer_sign));
    int8_t bad[FC_NUM_MOTORS][3];
    memcpy(bad, P.airframe.mixer_sign, sizeof(bad));
    bad[0][2] = -bad[0][2];                  /* ヨーの対角が崩れる */
    CHECK(!mixer_sign_table_valid(bad));
    memcpy(bad, P.airframe.mixer_sign, sizeof(bad));
    bad[1][0] = 0;
    CHECK(!mixer_sign_table_valid(bad));
    /* 対角（M1とM4、M2とM3）が同じ回転方向 = ヨーの符号が同じ */
    CHECK(P.airframe.mixer_sign[0][2] == P.airframe.mixer_sign[3][2]);
    CHECK(P.airframe.mixer_sign[1][2] == P.airframe.mixer_sign[2][2]);
    CHECK(P.airframe.mixer_sign[0][2] != P.airframe.mixer_sign[1][2]);
    CHECK(params_validate(&P));
}

static void test_mixer_signs_and_sum(void)
{
    mixer_result_t r;
    const airframe_params_t *af = &P.airframe;
    /* ロール+2g: 符号表どおりに ±2g、4つの合計は変わらない */
    controller_output_t u = U(10, 2, 0, 0);
    mixer_mix(af, &u, MAXG, &r);
    for (int i = 0; i < 4; i++) CHECK_NEAR(r.motor_g[i], 10.0f + 2.0f * af->mixer_sign[i][0], 1e-5f);
    CHECK(!r.saturated);
    u = U(10, 0, 3, 0);
    mixer_mix(af, &u, MAXG, &r);
    for (int i = 0; i < 4; i++) CHECK_NEAR(r.motor_g[i], 10.0f + 3.0f * af->mixer_sign[i][1], 1e-5f);
    u = U(10, 0, 0, 1);   /* ヨー: 対角の2つが上がり、残りが下がる */
    mixer_mix(af, &u, MAXG, &r);
    CHECK_NEAR(r.motor_g[0], r.motor_g[3], 1e-5f);   /* M1とM4（時計回り） */
    CHECK_NEAR(r.motor_g[1], r.motor_g[2], 1e-5f);   /* M2とM3（反時計回り） */
    CHECK_NEAR(r.motor_g[0] + r.motor_g[1] + r.motor_g[2] + r.motor_g[3], 40.0f, 1e-4f);
    CHECK(r.motor_g[0] != r.motor_g[1]);
    u = U(10, 1, 2, 3);   /* 3軸まとめて。重ね合わせ */
    mixer_mix(af, &u, MAXG, &r);
    for (int i = 0; i < 4; i++)
        CHECK_NEAR(r.motor_g[i], 10.0f + af->mixer_sign[i][0] * 1 + af->mixer_sign[i][1] * 2 + af->mixer_sign[i][2] * 3, 1e-5f);
}

static void test_mixer_saturation(void)
{
    mixer_result_t r;
    const airframe_params_t *af = &P.airframe;
    float sum;

    /* (1) ヨーだけ削る: T=28, ヨー5 → 上は30まで。収まる最大の割合 2/5=0.4 */
    controller_output_t u = U(28, 0, 0, 5);
    mixer_mix(af, &u, MAXG, &r);
    CHECK_NEAR(r.yaw_scale, 0.4f, 1e-5f);
    CHECK_NEAR(r.shift_g, 0.0f, 0.0f);
    CHECK_NEAR(r.rp_scale, 1.0f, 0.0f);
    CHECK(r.saturated);
    for (int i = 0; i < 4; i++) CHECK_NEAR(r.motor_g[i], 28.0f + 2.0f * af->mixer_sign[i][2], 1e-4f);

    /* (2) ロール・ピッチだけで一部がはみ出す: ヨーは0まで削り、4つを同じだけ下げる（全体推力を犠牲に） */
    u = U(28, 5, 0, 1);
    mixer_mix(af, &u, MAXG, &r);
    CHECK_NEAR(r.yaw_scale, 0.0f, 0.0f);
    CHECK_NEAR(r.shift_g, -3.0f, 1e-5f);           /* 28+5=33 → 30 */
    CHECK_NEAR(r.rp_scale, 1.0f, 0.0f);
    for (int i = 0; i < 4; i++) CHECK_NEAR(r.motor_g[i], 25.0f + 5.0f * af->mixer_sign[i][0], 1e-4f);
    /* ロールの差（右のM1と左のM3の差10g）は守られている */
    CHECK_NEAR(fabsf(r.motor_g[0] - r.motor_g[2]), 10.0f, 1e-4f);
    sum = r.motor_g[0] + r.motor_g[1] + r.motor_g[2] + r.motor_g[3];
    CHECK_NEAR(sum, 4.0f * 25.0f, 1e-3f);

    /* 下側（0gを割る）: T=2, ロール5 → 上へ3ずらす */
    u = U(2, 5, 0, 0);
    mixer_mix(af, &u, MAXG, &r);
    CHECK_NEAR(r.shift_g, 3.0f, 1e-5f);
    for (int i = 0; i < 4; i++) CHECK_NEAR(r.motor_g[i], 5.0f + 5.0f * af->mixer_sign[i][0], 1e-4f);

    /* (3) モーター同士の差が30gを超えて収まらない: ロール20+ピッチ20 → 差80g。ロール・ピッチを縮める（比率は保つ） */
    u = U(15, 20, 20, 0);
    mixer_mix(af, &u, MAXG, &r);
    CHECK_NEAR(r.rp_scale, 30.0f / 80.0f, 1e-5f);
    CHECK_NEAR(r.shift_g, 0.0f, 1e-4f);
    for (int i = 0; i < 4; i++) {
        CHECK(r.motor_g[i] >= 0.0f && r.motor_g[i] <= MAXG);
        CHECK_NEAR(r.motor_g[i], 15.0f + 0.375f * (20.0f * af->mixer_sign[i][0] + 20.0f * af->mixer_sign[i][1]), 1e-4f);
    }

    /* 推力が最大を超えて要求された場合は max に丸めて配る */
    u = U(100, 0, 0, 0);
    mixer_mix(af, &u, MAXG, &r);
    for (int i = 0; i < 4; i++) CHECK_NEAR(r.motor_g[i], MAXG, 1e-5f);

    /* NaN は全部0 */
    u = U(10, NAN, 0, 0);
    mixer_mix(af, &u, MAXG, &r);
    for (int i = 0; i < 4; i++) CHECK_NEAR(r.motor_g[i], 0.0f, 0.0f);
}

static void test_mixer_fuzz(void)
{
    /* どんな入力でも出力は 0..max。収まる入力ならロール・ピッチ・ヨーが指令どおり（飽和なし） */
    mixer_result_t r;
    const airframe_params_t *af = &P.airframe;
    int bad_range = 0, bad_unsat = 0, bad_rp = 0;
    for (int n = 0; n < 20000; n++) {
        controller_output_t u = U(30.0f * (frand() * 0.6f + 0.5f), 12.0f * frand(), 12.0f * frand(), 12.0f * frand());
        mixer_mix(af, &u, MAXG, &r);
        for (int i = 0; i < 4; i++) if (!(r.motor_g[i] >= 0.0f && r.motor_g[i] <= MAXG)) bad_range++;
        if (!r.saturated) {
            for (int i = 0; i < 4; i++) {
                float want = u.thrust_g + af->mixer_sign[i][0] * u.torque_g[0] + af->mixer_sign[i][1] * u.torque_g[1] +
                             af->mixer_sign[i][2] * u.torque_g[2];
                if (fabsf(want - r.motor_g[i]) > 1e-3f) bad_unsat++;
            }
        }
        /* rp_scale==1 のとき、ロール・ピッチ成分（モーター差）は指令どおり */
        if (r.rp_scale == 1.0f && u.thrust_g <= MAXG) {
            float roll_out = 0.25f * (af->mixer_sign[0][0] * r.motor_g[0] + af->mixer_sign[1][0] * r.motor_g[1] +
                                      af->mixer_sign[2][0] * r.motor_g[2] + af->mixer_sign[3][0] * r.motor_g[3]);
            if (fabsf(roll_out - u.torque_g[0]) > 1e-3f) bad_rp++;
        }
    }
    CHECK(bad_range == 0);
    CHECK(bad_unsat == 0);
    CHECK(bad_rp == 0);
}

/* ---------- 推力→デューティ ---------- */
static void test_thrust_model(void)
{
    const airframe_params_t *af = &P.airframe;
    /* 式: デューティ = 推力 ÷ (9.4 × 電圧) */
    CHECK_NEAR(thrust_model_to_duty(af, 20.0f, 3.8f), 20.0f / (9.4f * 3.8f), 1e-6f);
    CHECK_NEAR(thrust_model_to_duty(af, 20.0f, 4.2f), 20.0f / (9.4f * 4.2f), 1e-6f);
    CHECK_NEAR(thrust_model_to_duty(af, 20.0f, 3.5f), 0.6079f, 1e-3f);
    /* 電圧が下がると同じ推力に大きいデューティが要る（補償） */
    CHECK(thrust_model_to_duty(af, 20.0f, 3.5f) > thrust_model_to_duty(af, 20.0f, 4.2f));
    /* 逆算: max の推力を出すデューティは 1.0 */
    CHECK_NEAR(thrust_model_max_g(af, 4.2f), 9.4f * 4.2f, 1e-4f);
    CHECK_NEAR(thrust_model_to_duty(af, thrust_model_max_g(af, 3.9f), 3.9f), 1.0f, 1e-5f);
    /* 丸め */
    CHECK_NEAR(thrust_model_to_duty(af, 1000.0f, 3.8f), 1.0f, 0.0f);
    CHECK_NEAR(thrust_model_to_duty(af, -5.0f, 3.8f), 0.0f, 0.0f);
    CHECK_NEAR(thrust_model_to_duty(af, NAN, 3.8f), 0.0f, 0.0f);
    /* 電圧が0・NaN でも割り算が暴れない（下限に丸める） */
    CHECK_NEAR(thrust_model_to_duty(af, 20.0f, 0.0f), 20.0f / (9.4f * 3.0f), 1e-6f);
    CHECK_NEAR(thrust_model_to_duty(af, 20.0f, NAN), 20.0f / (9.4f * 3.0f), 1e-6f);
}

/* ---------- 全体の流れ（姿勢→PID→ミキサー→デューティ）の1周 ---------- */
static void test_chain(void)
{
    attitude_estimator_t e;
    attitude_init(&e, &P.attitude);
    vec3f_t zero = {0, 0, 0};
    float a10 = 10.0f * DEG;
    vec3f_t tilted = { 0, sinf(a10), cosf(a10) };   /* ロール +10° */
    run_batches(&e, BATCHES_PER_S * 4, zero, tilted, 0.0f);
    pid_controller_t c;
    controller_pid_init(&c, &P.pid);
    controller_t h = controller_pid_handle(&c);
    euler_t tgt = {0, 0, 0};
    controller_output_t prev = {0, {0, 0, 0}}, out;
    controller_input_t in;
    controller_input_make(&e.out, &tgt, 12.0f, &prev, &in);
    CHECK(h.ops->step(h.self, &in, 1.0f / 940.0f, &out));
    mixer_result_t mr;
    float vmax = thrust_model_max_g(&P.airframe, 3.8f);
    mixer_mix(&P.airframe, &out, vmax, &mr);
    float duty[4];
    for (int i = 0; i < 4; i++) duty[i] = thrust_model_to_duty(&P.airframe, mr.motor_g[i], 3.8f);
    /* ロール+側にずれている → ロール指令は負 → 符号表で roll 符号が+のモーターが下がる（戻す向き） */
    CHECK(out.torque_g[0] < 0.0f);
    for (int i = 0; i < 4; i++) {
        CHECK(duty[i] >= 0.0f && duty[i] <= 1.0f);
        if (P.airframe.mixer_sign[i][0] > 0) CHECK(mr.motor_g[i] < 12.0f);
        else CHECK(mr.motor_g[i] > 12.0f);
    }
}


static void test_battery_update_with_divider(void)
{
    battery_state_t b;
    fake_batt_init_calls = 0;
    battery_init(&b);
    CHECK(fake_batt_init_calls == 1);          /* 初期化は batt_adc_init を呼ぶ */
    /* 生カウント 2048 → 2048/4095 × vref × 分圧比。最初の1回はローパスを通さず採用する */
    fake_batt_raw = 2048;
    battery_update(&b, &P.battery, 1000);
    float expect = 2048.0f * (P.battery.adc_vref_v / 4095.0f) * P.battery.adc_divider_ratio;
    CHECK_NEAR(b.raw_v, expect, 1.0e-4f);
    CHECK_NEAR(b.voltage_v, expect, 1.0e-4f);
    CHECK(b.millivolt == (uint16_t)(expect * 1000.0f));
    /* フルスケール（4095）は vref × 分圧比 */
    fake_batt_raw = 4095;
    battery_update(&b, &P.battery, 1001);
    CHECK_NEAR(b.raw_v, P.battery.adc_vref_v * P.battery.adc_divider_ratio, 1.0e-4f);
}

static void test_cycle_timing(void)
{
    cycle_timing_t t = {0};
    fake_now_us = 10300;                        /* INT1(10000)から300µs遅れて処理開始 */
    cycle_timing_begin(&t, 10000, 1);
    CHECK(t.cycle_index == 1);
    CHECK(t.start_us == 10000);
    CHECK(t.t_begin_us == 10300);
    CHECK(t.wake_latency_us == 300);
    CHECK(t.int1_missed == 0);
    fake_now_us = 10300 + 640;
    cycle_timing_end(&t);
    CHECK(t.proc_us == 640);

    fake_now_us = 20100;                        /* INT1が3回ぶん進んだ → 取りこぼし2 */
    cycle_timing_begin(&t, 20000, 3);
    CHECK(t.cycle_index == 2);
    CHECK(t.int1_missed == 2);
    CHECK(t.wake_latency_us == 100);

    fake_now_us = 100000;                       /* 遅れが65535超 → 飽和 */
    cycle_timing_begin(&t, 0, 100000);
    CHECK(t.wake_latency_us == 65535);
    CHECK(t.int1_missed == 65535);
    fake_now_us = 100000 + 70000;
    cycle_timing_end(&t);
    CHECK(t.proc_us == 65535);

    fake_now_us = 5;                            /* 32bitの一周をまたいでも差分は正しい */
    cycle_timing_begin(&t, 0xFFFFFFF0u, 1);
    CHECK(t.wake_latency_us == 21);
}

/* imu_mount: 仕組みの試験。既定値でも実機の値でもない、試験用の任意の並べ替え {-3,1,2} を使う
 * （機体x = -基板z、機体y = +基板x、機体z = +基板y） */
static void test_imu_mount(void)
{
    imu_mount_params_t m = { { -3, 1, 2 } };
    CHECK(imu_mount_valid(&m));

    imu_batch_t b;
    memset(&b, 0, sizeof(b));
    b.count = 2;
    b.samples[0].accel_g.z = 1.0f;
    b.samples[1].accel_g.x = 1.0f;
    b.samples[1].gyro_rps.y = 2.0f;
    b.samples[1].gyro_rps.z = 3.0f;
    imu_mount_apply(&m, &b);
    CHECK_NEAR(b.samples[0].accel_g.x, -1.0f, 1e-6f);   /* 基板z → 機体 -x */
    CHECK_NEAR(b.samples[0].accel_g.y, 0.0f, 1e-6f);
    CHECK_NEAR(b.samples[0].accel_g.z, 0.0f, 1e-6f);
    CHECK_NEAR(b.samples[1].accel_g.y, 1.0f, 1e-6f);    /* 基板x → 機体 +y */
    CHECK_NEAR(b.samples[1].gyro_rps.x, -3.0f, 1e-6f);  /* 基板z → 機体 -x */
    CHECK_NEAR(b.samples[1].gyro_rps.y, 0.0f, 1e-6f);
    CHECK_NEAR(b.samples[1].gyro_rps.z, 2.0f, 1e-6f);   /* 基板y → 機体 +z */

    /* 恒等なら変えない。count より後ろには触らない */
    imu_mount_params_t id = { { 1, 2, 3 } };
    b.samples[2].gyro_rps.x = 5.0f;
    imu_mount_apply(&id, &b);
    CHECK_NEAR(b.samples[1].accel_g.y, 1.0f, 1e-6f);
    imu_mount_apply(&m, &b);
    CHECK_NEAR(b.samples[2].gyro_rps.x, 5.0f, 1e-6f);

    /* 不正: 0、範囲外、重複 → 弾く。不正なら apply は何もしない */
    imu_mount_params_t bad0 = { { 0, 2, 3 } }, bad4 = { { 1, 2, 4 } },
                       dup = { { 1, 1, 3 } }, dup2 = { { 2, -2, 3 } }, negmin = { { -128, 2, 3 } };
    CHECK(!imu_mount_valid(&bad0));
    CHECK(!imu_mount_valid(&bad4));
    CHECK(!imu_mount_valid(&dup));
    CHECK(!imu_mount_valid(&dup2));
    CHECK(!imu_mount_valid(&negmin));
    b.samples[1].gyro_rps.x = 7.0f;
    imu_mount_apply(&dup, &b);
    CHECK_NEAR(b.samples[1].gyro_rps.x, 7.0f, 1e-6f);

    /* params_validate は不正な並べ替えを弾き、有効な並べ替えは通す。既定値は恒等のまま */
    params_t p;
    params_set_defaults(&p);
    CHECK(params_validate(&p));
    p.imu_mount = m;
    CHECK(params_validate(&p));
    p.imu_mount = dup;
    CHECK(!params_validate(&p));
}

/* 確定した配置（設計メモ 2026-11-08）で、物理的な向きが合っているか。
 * 基板の軸で作った加速度 → axis_map → 姿勢 → PID → ミキサー で、傾きを戻す向きのモーターが上がるか */
static void physical_tilt(float board_ax, float board_ay, float board_az, attitude_state_t *out_as)
{
    attitude_estimator_t a;
    attitude_init(&a, &P.attitude);
    imu_batch_t b; memset(&b, 0, sizeof(b));
    b.count = 8;
    for (int k = 0; k < 8; k++) { b.samples[k].accel_g.x = board_ax; b.samples[k].accel_g.y = board_ay; b.samples[k].accel_g.z = board_az; }
    for (int n = 0; n < 4000; n++) {   /* 約4.3s。τ=0.5s の相補フィルタが収束する */
        imu_batch_t c = b;
        imu_mount_apply(&P.imu_mount, &c);
        attitude_update_batch(&a, &c, 1.0f / 7520.0f);
    }
    *out_as = *attitude_get(&a);
}

static void test_physical_signs(void)
{
    const float s20 = sinf(20.0f * 3.14159265f / 180.0f), c20 = cosf(20.0f * 3.14159265f / 180.0f);
    const airframe_params_t *af = &P.airframe;
    pid_controller_t pid; controller_pid_init(&pid, &P.pid);
    controller_t ctrl = controller_pid_handle(&pid);
    euler_t tgt = { 0.0f, 0.0f, 0.0f };
    controller_output_t prev = { 0.0f, { 0.0f, 0.0f, 0.0f } }, out;
    controller_input_t in;
    mixer_result_t r;
    attitude_state_t as;

    /* 左側を上げる（基板xは右向き。右が下がると、上向きの加速度が基板の -x 側へ傾く） */
    physical_tilt(-s20, 0.0f, c20, &as);
    CHECK(as.angle.roll_rad > 0.3f);                         /* roll+（左上げ） */
    CHECK(fabsf(as.angle.pitch_rad) < 0.02f);
    ctrl.ops->reset(ctrl.self);
    controller_input_make(&as, &tgt, 10.0f, &prev, &in);
    CHECK(ctrl.ops->step(ctrl.self, &in, 0.001f, &out));
    CHECK(out.torque_g[0] < 0.0f);                           /* 戻す = 負のロール */
    mixer_mix(af, &out, 30.0f, &r);
    CHECK(r.motor_g[2] < r.motor_g[0] && r.motor_g[3] < r.motor_g[1]);   /* 左のM3,M4が下がり、右のM1,M2が上がる */

    /* 機首を下げる（基板yは前向き。前が下がると、上向きの加速度が基板の -y 側へ傾く） */
    physical_tilt(0.0f, -s20, c20, &as);
    CHECK(as.angle.pitch_rad > 0.3f);                        /* pitch+（機首下げ） */
    CHECK(fabsf(as.angle.roll_rad) < 0.02f);
    ctrl.ops->reset(ctrl.self);
    controller_input_make(&as, &tgt, 10.0f, &prev, &in);
    CHECK(ctrl.ops->step(ctrl.self, &in, 0.001f, &out));
    CHECK(out.torque_g[1] < 0.0f);
    mixer_mix(af, &out, 30.0f, &r);
    CHECK(r.motor_g[1] > r.motor_g[0] && r.motor_g[3] > r.motor_g[2]);   /* 前のM2,M4が上がり、後ろのM1,M3が下がる */

    /* ヨー+（上から見て反時計回り）の指令は、時計回りのM1,M4を上げる（反トルクは反時計回り） */
    controller_output_t yaw = { 10.0f, { 0.0f, 0.0f, 1.0f } };
    mixer_mix(af, &yaw, 30.0f, &r);
    CHECK(r.motor_g[0] > r.motor_g[1] && r.motor_g[3] > r.motor_g[2]);
}

int main(void)
{
    params_set_defaults(&P);
    static const test_t tests[] = {
        { "attitude: 静止でバイアス収束・姿勢0・固定", test_attitude_still_and_bias },
        { "attitude: 動いている間は静止にしない", test_attitude_not_still_when_moving },
        { "attitude: 傾けた加速度にτで追従", test_attitude_tilt_follows_tau },
        { "attitude: 回転にジャイロで追従", test_attitude_gyro_tracks_rotation },
        { "attitude: ヨーは積分して折り返す", test_attitude_yaw_integrates_and_wraps },
        { "controller: ±180折り返しと入力14の並び", test_wrap_and_input_make },
        { "pid: 1段の P と出力上限", test_pid_axis },
        { "pid: D項はヨー±180°で跳ねない", test_pid_d_wraps_at_180 },
        { "pid: 積分の条件・上限・reset", test_pid_integral_conditions },
        { "pid: 2段の符号と軸の独立", test_pid_cascade_signs },
        { "mixer: 符号表の検査", test_mixer_sign_table },
        { "mixer: 符号と合計", test_mixer_signs_and_sum },
        { "mixer: 飽和（ヨー削り→ずらし→縮小）", test_mixer_saturation },
        { "mixer: 乱数入力で範囲と素通し", test_mixer_fuzz },
        { "imu_mount: 軸の並べ替えと不正な設定の拒否", test_imu_mount },
        { "physical: 傾けると戻す向きのモーターが上がる（確定した配置）", test_physical_signs },
        { "thrust: 推力→デューティの式", test_thrust_model },
        { "chain: 姿勢→PID→ミキサー→デューティ", test_chain },
        { "battery: 偽ADCの生カウント→電圧（分圧比）", test_battery_update_with_divider },
        { "cycle_timing: 遅れ・処理時間・取りこぼし", test_cycle_timing },
    };
    const test_t *suites[] = { tests, g_tests_fsm, g_tests_guard, g_tests_descent, g_tests_ipc, g_tests_loop };
    const int counts[] = { (int)(sizeof(tests) / sizeof(tests[0])), g_tests_fsm_n, g_tests_guard_n,
                           g_tests_descent_n, g_tests_ipc_n, g_tests_loop_n };
    int n = 0, failed_tests = 0;
    for (int s = 0; s < 6; s++) {
        for (int i = 0; i < counts[s]; i++) {
            int before = g_fail;
            g_test_checks = 0;
            suites[s][i].fn();
            bool ok = (g_fail == before);
            if (!ok) failed_tests++;
            n++;
            printf("%s  %s (%d checks)\n", ok ? "PASS" : "FAIL", suites[s][i].name, g_test_checks);
        }
    }
    if (g_fail == 0) {
        printf("\nALL PASS: %d tests, %d checks\n", n, g_checks);
        return 0;
    }
    printf("\nFAIL: %d of %d tests failed (%d checks failed)\n", failed_tests, n, g_fail);
    return 1;
}
