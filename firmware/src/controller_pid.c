/**
 * @file controller_pid.c
 * @brief 2段PID（controller_pid.h）。
 */
#include <math.h>
#include <string.h>
#include "controller_pid.h"
#include "attitude.h"

#define TWO_PI_F 6.28318530717959f

static float clampf(float v, float lim)
{
    if (v > lim) return lim;
    if (v < -lim) return -lim;
    return v;
}

float pid_axis_update(pid_axis_state_t *st, const pid_gains_t *g, float err, float meas,
                      float dt_s, bool integ_enable)
{
    /* 積分: 条件を満たす間だけためる。I項の寄与そのものを上限で止める（ki=0でも/後でkiを上げても暴れない） */
    if (integ_enable) {
        st->integ = clampf(st->integ + g->ki * err * dt_s, g->i_limit);
    }

    /* 微分は測定値の変化に対して取る（目標が急に変わっても跳ねない）。LPFで雑音を落とす */
    float d_term = 0.0f;
    if (g->kd > 0.0f) {
        if (st->primed) {
            /* 変化量は ±π に折り返す（ヨー角が +180°→-180° をまたいでも D が跳ねない。
             * roll/pitch は1周期で π も動かないので影響なし） */
            float raw = -attitude_wrap_pi(meas - st->prev_meas) / dt_s;
            float a = 1.0f;
            if (g->d_lpf_hz > 0.0f) {
                float rc = 1.0f / (TWO_PI_F * g->d_lpf_hz);
                a = dt_s / (dt_s + rc);
            }
            st->d_filt += a * (raw - st->d_filt);
        }
        d_term = g->kd * st->d_filt;
    }
    st->prev_meas = meas;
    st->primed = true;

    return clampf(g->kp * err + st->integ + d_term, g->out_limit);
}

static void pid_reset(void *self)
{
    pid_controller_t *c = (pid_controller_t *)self;
    memset(c->angle, 0, sizeof(c->angle));
    memset(c->rate, 0, sizeof(c->rate));
    memset(c->last_rate_target_rps, 0, sizeof(c->last_rate_target_rps));
}

static bool pid_step(void *self, const controller_input_t *in, float dt_s, controller_output_t *out)
{
    pid_controller_t *c = (pid_controller_t *)self;
    if (!(dt_s > 0.0f)) return false;
    const pid_params_t *p = c->params;

    const float att[3] = { in->attitude.roll_rad, in->attitude.pitch_rad, in->attitude.yaw_rad };
    const float tgt[3] = { in->target_attitude.roll_rad, in->target_attitude.pitch_rad, in->target_attitude.yaw_rad };
    const float rate[3] = { in->rate_rps.x, in->rate_rps.y, in->rate_rps.z };
    /* 積分は目標推力がしきい値を超えているときだけ（地上で積分がたまって離陸時に傾くのを防ぐ） */
    bool integ_en = in->target_thrust_g > p->i_enable_thrust_g;

    for (int i = 0; i < 3; i++) {
        float e_ang = tgt[i] - att[i];
        if (i == 2) e_ang = attitude_wrap_pi(e_ang);   /* ヨーは最短回りで */
        float rate_target = pid_axis_update(&c->angle[i], &p->angle[i], e_ang, att[i], dt_s, integ_en);
        c->last_rate_target_rps[i] = rate_target;
        out->torque_g[i] = pid_axis_update(&c->rate[i], &p->rate[i], rate_target - rate[i], rate[i], dt_s, integ_en);
    }
    out->thrust_g = in->target_thrust_g;
    return true;
}

static const controller_ops_t s_pid_ops = {
    .name = "pid", .id = CONTROLLER_PID, .reset = pid_reset, .step = pid_step,
};

void controller_pid_init(pid_controller_t *c, const pid_params_t *params)
{
    c->params = params;
    pid_reset(c);
}

controller_t controller_pid_handle(pid_controller_t *c)
{
    controller_t h = { &s_pid_ops, c };
    return h;
}
