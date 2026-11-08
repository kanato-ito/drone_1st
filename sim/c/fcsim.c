/**
 * @file fcsim.c
 * @brief ファームの1周期をPCで回す（fcsim.h）。
 *
 * ★仮★ fcsim_step の中身は firmware/apps/control_bench.c と同じ順に③aの信号の流れを呼んでいる。
 *   ステップ③bで control_loop_run_once(c, ev) がファームに入ったら、ここをそれ1つの呼び出しに置き換え、
 *   アーム要求・目標はメールボックス（本物）へ書く形にする。Python側の呼び方は変えない。
 *   control_bench と違う点: PWMは常に出す（シミュレータなのでデューティ上限なし）。
 */
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#include "fc_types.h"
#include "params.h"
#include "imu_lsm6dsv16x.h"
#include "attitude.h"
#include "imu_mount.h"
#include "controller.h"
#include "controller_pid.h"
#include "mixer.h"
#include "thrust_model.h"
#include "battery.h"
#include "motor_pwm.h"
#include "cycle_timer.h"
#include "fcsim.h"

void port_sim_reset_clock(void);   /* port_sim.c */

#define BATT_EVERY_N 8u   /* control_bench と同じ: 電池ADCは数周期に1回 */

static struct {
    imu_batch_t          batch;
    attitude_estimator_t att;
    pid_controller_t     pid;
    controller_t         ctrl;
    battery_state_t      batt;
    mixer_result_t       mix;
    controller_output_t  out;
    controller_output_t  prev_out;
    euler_t              target;        /* yaw はアーム時の向きからのずれ */
    float                target_thrust_g;
    float                yaw_at_arm;
    bool                 arm_request;
    bool                 armed;
    bool                 tripped;
    uint32_t             cycle;
    float                dt_sample;
    fcsim_telemetry_t    tel;
} S;

int32_t fcsim_init(void)
{
    memset(&S, 0, sizeof(S));
    port_sim_reset_clock();

    params_t *prm = params_get_mutable();
    params_set_defaults(prm);
    if (!params_validate(prm)) return -1;

    motor_pwm_init();
    motor_pwm_stop();
    imu_config_t cfg = imu_config_default();
    if (imu_init(&cfg) != IMU_OK) return -2;

    attitude_init(&S.att, &prm->attitude);
    controller_pid_init(&S.pid, &prm->pid);
    S.ctrl = controller_pid_handle(&S.pid);
    battery_init(&S.batt);
    S.dt_sample = imu_sample_period_s();
    return 0;
}

void fcsim_set_target(float roll_rad, float pitch_rad, float yaw_rad, float thrust_g)
{
    S.target.roll_rad = roll_rad;
    S.target.pitch_rad = pitch_rad;
    S.target.yaw_rad = yaw_rad;
    S.target_thrust_g = thrust_g;
}

static void disarm(void)
{
    S.armed = false;
    S.tripped = false;
    motor_pwm_stop();
    attitude_set_bias_locked(&S.att, false);
}

void fcsim_set_arm_request(int32_t arm)
{
    S.arm_request = arm != 0;
    if (!S.arm_request && S.armed) disarm();
}

static void fill_telemetry(void)
{
    const attitude_state_t *as = attitude_get(&S.att);
    fcsim_telemetry_t *t = &S.tel;
    t->armed = S.armed;
    t->tripped = S.tripped;
    t->still = attitude_is_still(&S.att);
    t->bias_ready = attitude_bias_ready(&S.att);
    t->batch_count = S.batch.count;
    t->cycle = S.cycle;
    t->angle_rad[0] = as->angle.roll_rad; t->angle_rad[1] = as->angle.pitch_rad; t->angle_rad[2] = as->angle.yaw_rad;
    t->rate_rps[0] = as->rate_rps.x; t->rate_rps[1] = as->rate_rps.y; t->rate_rps[2] = as->rate_rps.z;
    t->accel_g[0] = as->accel_g.x; t->accel_g[1] = as->accel_g.y; t->accel_g[2] = as->accel_g.z;
    t->bias_rps[0] = S.att.bias_rps.x; t->bias_rps[1] = S.att.bias_rps.y; t->bias_rps[2] = S.att.bias_rps.z;
    t->target_rad[0] = S.target.roll_rad; t->target_rad[1] = S.target.pitch_rad;
    t->target_rad[2] = attitude_wrap_pi(S.yaw_at_arm + S.target.yaw_rad);
    t->target_thrust_g = S.target_thrust_g;
    for (int i = 0; i < 3; i++) {
        t->rate_target_rps[i] = S.pid.last_rate_target_rps[i];
        t->out_torque_g[i] = S.out.torque_g[i];
        t->rate_integ_g[i] = S.pid.rate[i].integ;
    }
    t->out_thrust_g = S.out.thrust_g;
    for (int i = 0; i < FC_NUM_MOTORS; i++) t->motor_g[i] = S.mix.motor_g[i];
    t->mix_saturated = S.mix.saturated;
    t->mix_yaw_scale = S.mix.yaw_scale;
    t->mix_shift_g = S.mix.shift_g;
    t->mix_rp_scale = S.mix.rp_scale;
    t->batt_v = S.batt.voltage_v;
    t->batt_raw_v = S.batt.raw_v;
    fcsim_get_duty(t->duty);
}

int32_t fcsim_step(uint32_t int1_timestamp_us)
{
    (void)int1_timestamp_us;   /* 仮の1周期では使わない（③bの control_loop_run_once で ev として渡す） */
    const params_t *prm = params_get();

    if (imu_read_fifo(&S.batch) != IMU_OK || S.batch.count == 0) {
        fill_telemetry();
        return 0;
    }

    imu_mount_apply(&prm->imu_mount, &S.batch);   /* 基板の軸 → 機体の軸（control_bench と同じ位置） */
    attitude_update_batch(&S.att, &S.batch, S.dt_sample);
    const attitude_state_t *as = attitude_get(&S.att);

    if ((S.cycle % BATT_EVERY_N) == 0) {
        battery_update(&S.batt, &prm->battery, fc_clock_now_us32() / 1000u);
    }

    /* --- 待機 → アーム（control_bench の簡易手順 + アーム要求）--- */
    if (S.arm_request && !S.armed && attitude_is_still(&S.att) && attitude_bias_ready(&S.att) &&
        attitude_tilt_rad(as) <= prm->safety.arm_level_tol_rad) {
        attitude_set_bias_locked(&S.att, true);   /* アーム時にバイアス固定（rev22） */
        S.ctrl.ops->reset(S.ctrl.self);           /* アーム時に積分0（rev22） */
        S.yaw_at_arm = as->angle.yaw_rad;
        S.prev_out = (controller_output_t){ 0.0f, { 0.0f, 0.0f, 0.0f } };
        S.armed = true;
        S.tripped = false;
    }

    if (S.armed && !S.tripped) {
        euler_t tgt = S.target;
        tgt.yaw_rad = attitude_wrap_pi(S.yaw_at_arm + S.target.yaw_rad);
        controller_input_t in;
        controller_input_make(as, &tgt, S.target_thrust_g, &S.prev_out, &in);
        if (!S.ctrl.ops->step(S.ctrl.self, &in, S.dt_sample * (float)S.batch.count, &S.out)) {
            S.out = (controller_output_t){ 0.0f, { 0.0f, 0.0f, 0.0f } };
        }
        S.prev_out = S.out;

        float v = S.batt.voltage_v;
        float vmax = thrust_model_max_g(&prm->airframe, v);
        mixer_mix(&prm->airframe, &S.out, vmax, &S.mix);
        float duty[FC_NUM_MOTORS];
        for (int i = 0; i < FC_NUM_MOTORS; i++) {
            duty[i] = thrust_model_to_duty(&prm->airframe, S.mix.motor_g[i], v);
        }
        if (attitude_tilt_rad(as) > prm->safety.tilt_estop_rad) {
            S.tripped = true;   /* ディスアームまで保持 */
            motor_pwm_stop();
        } else {
            motor_pwm_set(duty);
        }
    } else {
        motor_pwm_stop();
    }

    fc_wdt_feed();
    S.cycle++;
    fill_telemetry();
    return S.batch.count;
}

void fcsim_get_telemetry(fcsim_telemetry_t *out) { *out = S.tel; }

int32_t fcsim_telemetry_size(void) { return (int32_t)sizeof(fcsim_telemetry_t); }

/* ---------------- パラメータ ---------------- */

static pid_gains_t *gains_ptr(params_t *p, int32_t stage, int32_t axis)
{
    if (axis < 0 || axis > 2) return NULL;
    if (stage == 0) return &p->pid.angle[axis];
    if (stage == 1) return &p->pid.rate[axis];
    return NULL;
}

int32_t fcsim_set_pid_gains(int32_t stage, int32_t axis, const float g[6])
{
    params_t *p = params_get_mutable();
    pid_gains_t *dst = gains_ptr(p, stage, axis);
    if (!dst) return -1;
    pid_gains_t old = *dst;
    dst->kp = g[0]; dst->ki = g[1]; dst->kd = g[2];
    dst->i_limit = g[3]; dst->out_limit = g[4]; dst->d_lpf_hz = g[5];
    if (!params_validate(p)) { *dst = old; return -1; }
    return 0;
}

int32_t fcsim_get_pid_gains(int32_t stage, int32_t axis, float g_out[6])
{
    pid_gains_t *src = gains_ptr(params_get_mutable(), stage, axis);
    if (!src) return -1;
    g_out[0] = src->kp; g_out[1] = src->ki; g_out[2] = src->kd;
    g_out[3] = src->i_limit; g_out[4] = src->out_limit; g_out[5] = src->d_lpf_hz;
    return 0;
}

int32_t fcsim_set_mixer_sign(const int8_t sign[12])
{
    int8_t tbl[FC_NUM_MOTORS][3];
    for (int m = 0; m < FC_NUM_MOTORS; m++)
        for (int a = 0; a < 3; a++) tbl[m][a] = sign[m * 3 + a];
    if (!mixer_sign_table_valid((const int8_t (*)[3])tbl)) return -1;
    memcpy(params_get_mutable()->airframe.mixer_sign, tbl, sizeof(tbl));
    return 0;
}

void fcsim_get_mixer_sign(int8_t sign_out[12])
{
    const params_t *p = params_get();
    for (int m = 0; m < FC_NUM_MOTORS; m++)
        for (int a = 0; a < 3; a++) sign_out[m * 3 + a] = p->airframe.mixer_sign[m][a];
}

typedef enum { PT_FLOAT, PT_U32 } ptype_t;
typedef struct { const char *name; ptype_t type; size_t offset; } pentry_t;

#define PF(name, field) { name, PT_FLOAT, offsetof(params_t, field) }
#define PU(name, field) { name, PT_U32, offsetof(params_t, field) }

/* 試験で変えたくなるものだけ。増やすときはここに1行足す */
static const pentry_t s_ptable[] = {
    PF("pid.i_enable_thrust_g",        pid.i_enable_thrust_g),
    PF("attitude.tau_s",               attitude.tau_s),
    PF("attitude.still_gyro_std_rps",  attitude.still_gyro_std_rps),
    PF("attitude.still_accel_tol_g",   attitude.still_accel_tol_g),
    PU("attitude.still_hold_ms",       attitude.still_hold_ms),
    PF("airframe.thrust_k_g_per_vduty", airframe.thrust_k_g_per_vduty),
    PF("battery.adc_divider_ratio",    battery.adc_divider_ratio),
    PF("battery.lpf_tau_s",            battery.lpf_tau_s),
    PF("battery.v_low",                battery.v_low),
    PF("safety.tilt_estop_rad",        safety.tilt_estop_rad),
    PF("safety.arm_level_tol_rad",     safety.arm_level_tol_rad),
};

static const pentry_t *find_param(const char *name)
{
    if (!name) return NULL;
    for (size_t i = 0; i < sizeof(s_ptable) / sizeof(s_ptable[0]); i++) {
        if (strcmp(s_ptable[i].name, name) == 0) return &s_ptable[i];
    }
    return NULL;
}

int32_t fcsim_set_param(const char *name, float value)
{
    const pentry_t *e = find_param(name);
    if (!e || !isfinite(value)) return -1;
    params_t *p = params_get_mutable();
    params_t old = *p;
    unsigned char *base = (unsigned char *)p;
    if (e->type == PT_FLOAT) {
        float *dst = (float *)(void *)(base + e->offset);
        *dst = value;
    } else {
        if (value < 0.0f) return -1;
        uint32_t *dst = (uint32_t *)(void *)(base + e->offset);
        *dst = (uint32_t)value;
    }
    if (!params_validate(p)) { *p = old; return -1; }
    return 0;
}

int32_t fcsim_get_param(const char *name, float *value_out)
{
    const pentry_t *e = find_param(name);
    if (!e) return -1;
    const unsigned char *base = (const unsigned char *)params_get();
    if (e->type == PT_FLOAT) {
        *value_out = *(const float *)(const void *)(base + e->offset);
    } else {
        *value_out = (float)*(const uint32_t *)(const void *)(base + e->offset);
    }
    return 0;
}
