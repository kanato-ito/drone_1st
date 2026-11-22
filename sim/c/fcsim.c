/**
 * @file fcsim.c
 * @brief ファームの1周期をPCで回す（fcsim.h）。
 *
 * 1周期は本物の control_loop_run_once（firmware/src/control_loop.c）をそのまま呼ぶ。
 * 実機の control_loop_step がしている「INT1を待つ」部分だけをここで演じる:
 *   - 呼び出し側（runner.py）が IMU のサンプルを8個積んでから fcsim_step(1) を呼ぶ = INT1が来た。
 *   - fcsim_step(0) は INT1が来ない周期（IMU途絶）= run_once(NULL)。
 * コア1の役（操縦パケット → メールボックス、ディスアーム・非常停止 → 旗）は fcsim_send_packet。
 * メールボックスと旗は本物（mailbox.c / ipc_flags.c）をリンクしている。
 */
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#include "fc_types.h"
#include "params.h"
#include "control_loop.h"
#include "mailbox.h"
#include "ipc_flags.h"
#include "imu_lsm6dsv16x.h"
#include "motor_pwm.h"
#include "fcsim.h"

/* port_sim.c */
void port_sim_reset_clock(void);
void fcsim_int1_edge(void);
void fcsim_get_int1_event(imu_int1_event_t *ev);

static control_loop_t   C;          /* コア0の全状態（実機と同じく静的に1つ） */
static fcsim_telemetry_t T;
static uint32_t          s_packet_seq;

int32_t fcsim_init(float batt_v)
{
    port_sim_reset_clock();

    params_t *prm = params_get_mutable();
    params_set_defaults(prm);
    if (!params_validate(prm)) return -1;

    /* 電源投入: ADCには最初から電池の電圧が出ている（battery のローパスの初期値になる） */
    fcsim_set_batt_voltage(batt_v);

    /* コア1を起動する前の ipc の初期化（実機では main がする）。メールボックスは未受信（seq 0）に戻す */
    ipc_flags_init(ipc_flags());
    mailbox_test_force_seq(0u);
    s_packet_seq = 0;

    control_loop_init(&C, prm);   /* IMU・PWM・電池・状態機械。最後にWDT開始 */
    memset(&T, 0, sizeof(T));
    return 0;
}

void fcsim_send_packet(float roll, float pitch, float yaw, float throttle, uint32_t flags)
{
    control_packet_t pkt;
    pkt.roll = roll;
    pkt.pitch = pitch;
    pkt.yaw = yaw;
    pkt.throttle = throttle;
    pkt.flags = flags;
    pkt.packet_seq = ++s_packet_seq;
    (void)mailbox_write(&pkt);
    /* コア1は旗付きパケットを受けるたびに旗も立てる（ipc_flags.h） */
    if (flags & CTRL_FLAG_DISARM) ipc_flags_request_disarm(ipc_flags());
    if (flags & CTRL_FLAG_ESTOP) ipc_flags_request_estop(ipc_flags());
}

static void fill_telemetry(void)
{
    const attitude_state_t *as = attitude_get(&C.att);
    fcsim_telemetry_t *t = &T;
    const flight_state_t st = C.fsm.state;
    t->state = (int32_t)st;
    t->estop_reason = (int32_t)C.fsm.estop_reason;
    t->descent_reason = (int32_t)C.fsm.descent_reason;
    t->armed = (st == FS_FLIGHT || st == FS_EMERGENCY_DESCENT);
    t->tripped = (st == FS_EMERGENCY_STOP);
    t->motors_on = C.motors_on;
    t->still = attitude_is_still(&C.att);
    t->bias_ready = attitude_bias_ready(&C.att);
    t->batch_count = C.batch.count;
    t->batt_low = C.batt.low;
    t->int1_stuck = C.int1_stuck;
    t->int1_pending = C.int1_pending;
    t->no_sample_count = C.no_sample_count;
    t->imu_fail_count = C.imu_fail_count;
    t->cycle = C.stats.cycles;
    t->angle_rad[0] = as->angle.roll_rad; t->angle_rad[1] = as->angle.pitch_rad; t->angle_rad[2] = as->angle.yaw_rad;
    t->rate_rps[0] = as->rate_rps.x; t->rate_rps[1] = as->rate_rps.y; t->rate_rps[2] = as->rate_rps.z;
    t->accel_g[0] = as->accel_g.x; t->accel_g[1] = as->accel_g.y; t->accel_g[2] = as->accel_g.z;
    t->bias_rps[0] = C.att.bias_rps.x; t->bias_rps[1] = C.att.bias_rps.y; t->bias_rps[2] = C.att.bias_rps.z;
    t->target_rad[0] = C.target_att.roll_rad; t->target_rad[1] = C.target_att.pitch_rad;
    t->target_rad[2] = C.target_att.yaw_rad;
    t->target_thrust_g = C.target_thrust_g;
    for (int i = 0; i < 3; i++) {
        t->rate_target_rps[i] = C.pid.last_rate_target_rps[i];
        t->out_torque_g[i] = C.out.torque_g[i];
        t->rate_integ_g[i] = C.pid.rate[i].integ;
    }
    t->out_thrust_g = C.out.thrust_g;
    for (int i = 0; i < FC_NUM_MOTORS; i++) t->motor_g[i] = C.motors_on ? C.mix.motor_g[i] : 0.0f;
    t->mix_saturated = C.motors_on && C.mix.saturated;
    t->mix_yaw_scale = C.mix.yaw_scale;
    t->mix_shift_g = C.mix.shift_g;
    t->mix_rp_scale = C.mix.rp_scale;
    t->batt_v = C.batt.voltage_v;
    t->batt_raw_v = C.batt.raw_v;
    fcsim_get_duty(t->duty);
}

int32_t fcsim_step(int32_t int1)
{
    bool pending;
    if (int1) {
        fcsim_int1_edge();   /* INT1の立ち上がり（割り込みが時刻と回数を記録するのと同じ） */
        imu_int1_event_t ev;
        fcsim_get_int1_event(&ev);
        pending = control_loop_run_once(&C, &ev);
    } else {
        pending = control_loop_run_once(&C, NULL);
    }
    fill_telemetry();
    return pending ? 1 : 0;
}

void fcsim_get_telemetry(fcsim_telemetry_t *out) { *out = T; }

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
    PF("battery.v_min",                battery.v_min),
    PF("safety.tilt_estop_rad",        safety.tilt_estop_rad),
    PF("safety.arm_level_tol_rad",     safety.arm_level_tol_rad),
    PF("safety.idle_throttle",         safety.idle_throttle),
    PU("safety.comm_timeout_ms",       safety.comm_timeout_ms),
    PU("battery.low_hold_ms",          battery.low_hold_ms),
    PF("airframe.throttle_full_thrust_g", airframe.throttle_full_thrust_g),
    PF("descent.thrust_ratio",         descent.thrust_ratio),
    PU("descent.max_time_ms",          descent.max_time_ms),
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
