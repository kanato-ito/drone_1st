/**
 * @file guard.c
 * @brief PIDの見張り役（guard.h）。取り上げ判定とラッチ。純粋C、SDK非依存。
 *
 * 判定は「!(値 <= しきい値)」の形で書き、NaN も異常側に倒す。
 */
#include <math.h>
#include "guard.h"

void guard_reset(guard_t *g)
{
    g->reason = TAKEOVER_NONE;
    g->sat_count = 0;
}

bool guard_taken_over(const guard_t *g)
{
    return g->reason != TAKEOVER_NONE;
}

void guard_force_takeover(guard_t *g, takeover_reason_t reason)
{
    if (g->reason == TAKEOVER_NONE && reason != TAKEOVER_NONE) g->reason = reason;
}

bool guard_check_pre(guard_t *g, const guard_params_t *gp, const attitude_state_t *att, bool human_takeover)
{
    if (guard_taken_over(g)) return true;   /* ラッチ: 戻さない */

    if (human_takeover) {
        g->reason = TAKEOVER_HUMAN;
    } else if (!(attitude_tilt_rad(att) <= gp->tilt_takeover_rad)) {
        g->reason = TAKEOVER_TILT;
    } else {
        const vec3f_t *w = &att->rate_rps;
        float rate = sqrtf(w->x * w->x + w->y * w->y + w->z * w->z);
        if (!(rate <= gp->rate_takeover_rps)) g->reason = TAKEOVER_RATE;
    }
    return guard_taken_over(g);
}

bool guard_check_post(guard_t *g, const guard_params_t *gp, const controller_output_t *o,
                      float thrust_max_g, float torque_max_g, uint32_t elapsed_us)
{
    if (guard_taken_over(g)) return true;

    /* 1. 出力が有限か */
    if (!isfinite(o->thrust_g) || !isfinite(o->torque_g[0]) || !isfinite(o->torque_g[1]) ||
        !isfinite(o->torque_g[2])) {
        g->reason = TAKEOVER_POLICY_NAN;
        return true;
    }
    /* 2. 計算時間が予算内か */
    if (elapsed_us > gp->policy_budget_us) {
        g->reason = TAKEOVER_POLICY_OVERRUN;
        return true;
    }
    /* 3. 上限への張りつきが続くか（どれかが上限の policy_sat_ratio 倍以上の周期を数える） */
    const float t_lim = gp->policy_sat_ratio * thrust_max_g;
    const float q_lim = gp->policy_sat_ratio * torque_max_g;
    bool sat = fabsf(o->thrust_g) >= t_lim;
    for (int i = 0; i < 3; i++) {
        if (fabsf(o->torque_g[i]) >= q_lim) sat = true;
    }
    if (sat) {
        if (g->sat_count < 0xFFFFu) g->sat_count++;
        if (g->sat_count >= gp->policy_sat_cycles) g->reason = TAKEOVER_POLICY_SATURATED;
    } else {
        g->sat_count = 0;
    }
    return guard_taken_over(g);
}

const char *takeover_reason_name(takeover_reason_t r)
{
    switch (r) {
    case TAKEOVER_NONE:             return "none";
    case TAKEOVER_TILT:             return "tilt";
    case TAKEOVER_RATE:             return "rate";
    case TAKEOVER_POLICY_NAN:       return "policy_nan";
    case TAKEOVER_POLICY_SATURATED: return "policy_saturated";
    case TAKEOVER_POLICY_OVERRUN:   return "policy_overrun";
    case TAKEOVER_HUMAN:            return "human";
    case TAKEOVER_POLICY_FAIL:      return "policy_fail";
    default:                        return "?";
    }
}
