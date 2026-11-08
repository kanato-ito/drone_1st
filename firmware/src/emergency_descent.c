/**
 * @file emergency_descent.c
 * @brief 非常時降下の目標生成と着地検出（emergency_descent.h）。純粋C、SDK非依存。
 *
 * 着地 = 衝撃ピーク(|a| > shock_g)を見たあと、静かな状態
 *        (|角速度| < quiet_rate_rps かつ | |a| - 1g | < quiet_accel_tol_g)が quiet_hold_ms 続くこと。
 * 衝撃を見る前の静かさは数えない（空中でゆっくり降りているだけかもしれないため）。
 * 動いたら静かな時間は数え直す。
 * 注意: attitude の accel_g は1周期（約8サンプル≒1ms）の平均なので、それより短い衝撃は鈍る。
 *       shock_g は仮で、飛行試験で決める。
 */
#include <math.h>
#include "emergency_descent.h"

void descent_init(descent_t *d)
{
    d->hover_thrust_est_g = 0.0f;
    d->hover_primed = false;
    d->start_us = 0;
    d->shock_seen = false;
    d->quiet_active = false;
    d->quiet_since_us = 0;
    d->yaw_hold_rad = 0.0f;
}

void descent_track_hover(descent_t *d, float thrust_cmd_g)
{
    if (!isfinite(thrust_cmd_g) || thrust_cmd_g < 0.0f) return;
    if (!d->hover_primed) {
        d->hover_thrust_est_g = thrust_cmd_g;
        d->hover_primed = true;
    } else {
        d->hover_thrust_est_g += DESCENT_HOVER_LPF_ALPHA * (thrust_cmd_g - d->hover_thrust_est_g);
    }
}

void descent_begin(descent_t *d, const attitude_state_t *att, uint64_t now_us)
{
    d->start_us = now_us;
    d->shock_seen = false;
    d->quiet_active = false;
    d->quiet_since_us = 0;
    d->yaw_hold_rad = att->angle.yaw_rad;
}

void descent_update(descent_t *d, const descent_params_t *p, const attitude_state_t *att,
                    uint64_t now_us, descent_target_t *out)
{
    /* 目標: 水平、ヨーは降下開始時、推力は直前のホバリングの ratio 倍 */
    out->target_attitude.roll_rad = 0.0f;
    out->target_attitude.pitch_rad = 0.0f;
    out->target_attitude.yaw_rad = d->yaw_hold_rad;
    out->target_thrust_g = p->thrust_ratio * d->hover_thrust_est_g;

    out->timed_out = (now_us - d->start_us) >= (uint64_t)p->max_time_ms * 1000u;

    const vec3f_t *a = &att->accel_g;
    const vec3f_t *w = &att->rate_rps;
    const float amag = sqrtf(a->x * a->x + a->y * a->y + a->z * a->z);
    const float rate = sqrtf(w->x * w->x + w->y * w->y + w->z * w->z);

    if (amag > p->shock_g) d->shock_seen = true;

    const bool quiet = (rate < p->quiet_rate_rps) && (fabsf(amag - 1.0f) < p->quiet_accel_tol_g);
    out->landed = false;
    if (d->shock_seen && quiet) {
        if (!d->quiet_active) {
            d->quiet_active = true;
            d->quiet_since_us = now_us;
        }
        if ((now_us - d->quiet_since_us) >= (uint64_t)p->quiet_hold_ms * 1000u) out->landed = true;
    } else {
        d->quiet_active = false;   /* 動いた（または衝撃前）: 数え直し */
    }
}
