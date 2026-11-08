/**
 * @file attitude.c
 * @brief 姿勢推定（attitude.h）。相補フィルタ + ヨー積分 + ジャイロのバイアス推定。
 *
 * 1周期（約8サンプル）ぶんをまとめて1回更新する。理由: サンプル毎にsinf/cosf/atan2fを
 * 呼ぶと約7.5kHz分の三角関数になる。バッチ平均の角速度で積分しても、8サンプル(約1ms)の間の
 * 長方形積分と同じ結果で、誤差は無視できる。
 */
#include <math.h>
#include <string.h>
#include "attitude.h"

#define PI_F 3.14159265358979f

/* 加速度の大きさが 1g からこれ以上ずれたら、その周期は加速度で補正しない（提案）。
 * 飛行中の加速・振動で傾き角を引きずられないため。静止判定のしきい値(params)とは別。 */
#define ACCEL_GATE_G 0.3f

/* cosθ の下限。ピッチ ±90° 付近で tanθ や 1/cosθ が発散するのを防ぐ（提案） */
#define COS_PITCH_MIN 0.05f

float attitude_wrap_pi(float rad)
{
    if (!isfinite(rad)) return 0.0f;
    /* 1回〜数回の折り返しで足りる値しか来ない前提だが、念のためfmodfで大きな値も扱う */
    if (rad > PI_F || rad <= -PI_F) {
        rad = fmodf(rad, 2.0f * PI_F);
        if (rad > PI_F) rad -= 2.0f * PI_F;
        else if (rad <= -PI_F) rad += 2.0f * PI_F;
    }
    return rad;
}

void attitude_init(attitude_estimator_t *e, const attitude_params_t *params)
{
    memset(e, 0, sizeof(*e));
    e->params = params;
    e->out.accel_g.z = 1.0f;
}

static void accel_to_angles(const vec3f_t *a, float *roll, float *pitch)
{
    /* 右手系・z上向き: 上向きベクトルを機体座標で見ると (-sinθ, cosθ sinφ, cosθ cosφ) */
    *roll  = atan2f(a->y, a->z);
    *pitch = atan2f(-a->x, sqrtf(a->y * a->y + a->z * a->z));
}

void attitude_align_from_accel(attitude_estimator_t *e, const imu_sample_t *s)
{
    accel_to_angles(&s->accel_g, &e->out.angle.roll_rad, &e->out.angle.pitch_rad);
    e->out.angle.yaw_rad = 0.0f;
    e->out.accel_g = s->accel_g;
    e->out.valid = true;
}

void attitude_set_bias_locked(attitude_estimator_t *e, bool locked) { e->bias_locked = locked; }
bool attitude_is_still(const attitude_estimator_t *e) { return e->still; }
bool attitude_bias_ready(const attitude_estimator_t *e) { return e->bias_ready; }
const attitude_state_t *attitude_get(const attitude_estimator_t *e) { return &e->out; }

float attitude_tilt_rad(const attitude_state_t *s)
{
    float c = cosf(s->angle.roll_rad) * cosf(s->angle.pitch_rad);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return acosf(c);
}

/** 静止判定とバイアス推定。g = バッチ平均のジャイロ（生）。 */
static void update_still_and_bias(attitude_estimator_t *e, const vec3f_t *g, float accel_norm, float bdt)
{
    const attitude_params_t *p = e->params;
    float alpha = bdt / (p->still_window_s + bdt);
    float thr2 = p->still_gyro_std_rps * p->still_gyro_std_rps;

    if (!e->stat_primed) {
        e->gyro_mean = *g;
        /* ばらつきは大きめから始める: 最初の1周期だけで「静止」と誤判定しないため */
        e->gyro_var.x = e->gyro_var.y = e->gyro_var.z = 4.0f * thr2;
        e->stat_primed = true;
    } else {
        float dx = g->x - e->gyro_mean.x, dy = g->y - e->gyro_mean.y, dz = g->z - e->gyro_mean.z;
        e->gyro_mean.x += alpha * dx;  e->gyro_mean.y += alpha * dy;  e->gyro_mean.z += alpha * dz;
        e->gyro_var.x += alpha * (dx * dx - e->gyro_var.x);
        e->gyro_var.y += alpha * (dy * dy - e->gyro_var.y);
        e->gyro_var.z += alpha * (dz * dz - e->gyro_var.z);
    }

    /* 一定の速さで回り続けると「ばらつき」は0になるので、平均の大きさにも上限を付ける（提案） */
    float amax = p->still_gyro_abs_max_rps;
    bool bias_plausible = fabsf(e->gyro_mean.x) <= amax && fabsf(e->gyro_mean.y) <= amax &&
                          fabsf(e->gyro_mean.z) <= amax;
    bool quiet = bias_plausible && e->gyro_var.x <= thr2 && e->gyro_var.y <= thr2 && e->gyro_var.z <= thr2 &&
                 fabsf(accel_norm - 1.0f) <= p->still_accel_tol_g;
    /* 1回でも外れたら数え直し（途中の動きを含んだ平均をバイアスにしないため） */
    e->still_time_s = quiet ? e->still_time_s + bdt : 0.0f;
    e->still = e->still_time_s * 1000.0f >= (float)p->still_hold_ms;

    if (e->still && !e->bias_locked) {
        if (!e->bias_ready) {
            e->bias_rps = e->gyro_mean;   /* 最初の確定では窓平均をそのまま採用して早く収束させる */
            e->bias_ready = true;
        } else {
            float b = bdt / (p->bias_tau_s + bdt);
            e->bias_rps.x += b * (g->x - e->bias_rps.x);
            e->bias_rps.y += b * (g->y - e->bias_rps.y);
            e->bias_rps.z += b * (g->z - e->bias_rps.z);
        }
    }
}

void attitude_update_batch(attitude_estimator_t *e, const imu_batch_t *batch, float dt_s)
{
    uint8_t n = batch->count;
    if (n == 0 || !(dt_s > 0.0f)) return;

    vec3f_t g = {0, 0, 0}, a = {0, 0, 0};
    for (uint8_t i = 0; i < n; i++) {
        g.x += batch->samples[i].gyro_rps.x;  g.y += batch->samples[i].gyro_rps.y;  g.z += batch->samples[i].gyro_rps.z;
        a.x += batch->samples[i].accel_g.x;   a.y += batch->samples[i].accel_g.y;   a.z += batch->samples[i].accel_g.z;
    }
    float inv = 1.0f / (float)n;
    g.x *= inv; g.y *= inv; g.z *= inv;
    a.x *= inv; a.y *= inv; a.z *= inv;
    float bdt = dt_s * (float)n;
    float anorm = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);

    if (!e->out.valid) {
        imu_sample_t s0 = { g, a };
        attitude_align_from_accel(e, &s0);
    }

    update_still_and_bias(e, &g, anorm, bdt);

    /* バイアスを引いた機体の角速度 */
    vec3f_t w = { g.x - e->bias_rps.x, g.y - e->bias_rps.y, g.z - e->bias_rps.z };

    /* 機体の角速度 (p,q,r) → オイラー角の変化率（Z-Y-X）。傾きが大きいときも成り立つ */
    float phi = e->out.angle.roll_rad, theta = e->out.angle.pitch_rad;
    float sphi = sinf(phi), cphi = cosf(phi);
    float ctheta = cosf(theta);
    if (ctheta < COS_PITCH_MIN) ctheta = COS_PITCH_MIN;
    float tan_theta = sinf(theta) / ctheta;
    float qr = w.y * sphi + w.z * cphi;
    float roll_dot  = w.x + qr * tan_theta;
    float pitch_dot = w.y * cphi - w.z * sphi;
    float yaw_dot   = qr / ctheta;

    float roll_pred  = phi + roll_dot * bdt;
    float pitch_pred = theta + pitch_dot * bdt;

    /* 相補フィルタ: 短い時間はジャイロ(alpha)、長い時間は加速度(1-alpha)。
     * 角度の差を折り返してから混ぜる: ±π をまたぐ平均が逆向きになるのを防ぐ */
    float roll = roll_pred, pitch = pitch_pred;
    if (fabsf(anorm - 1.0f) <= ACCEL_GATE_G) {
        float ar, ap;
        accel_to_angles(&a, &ar, &ap);
        float k = bdt / (e->params->tau_s + bdt);   /* = 1 - tau/(tau+dt) */
        roll  = roll_pred  + k * attitude_wrap_pi(ar - roll_pred);
        pitch = pitch_pred + k * (ap - pitch_pred);
    }

    e->out.angle.roll_rad  = attitude_wrap_pi(roll);
    e->out.angle.pitch_rad = pitch;
    /* ヨーはジャイロの積分だけ（磁気なし）。±πに折り返して値が増え続けないようにする */
    e->out.angle.yaw_rad   = attitude_wrap_pi(e->out.angle.yaw_rad + yaw_dot * bdt);
    e->out.rate_rps = w;
    e->out.accel_g = a;
}
