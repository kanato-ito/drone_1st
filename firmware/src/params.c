/**
 * @file params.c
 * @brief 既定値（すべて「仮」）。ステップ③aではゲインを小さめの安全側に置いてある。
 *
 * 試験台・実機で決める値ばかり。数値を変えるときはここだけ直す。SDK非依存。
 */
#include <math.h>
#include <string.h>
#include "params.h"
#include "mixer.h"
#include "imu_mount.h"

static params_t s_params;

/** NaN・負・非有限を弾く小道具。 */
static bool ok_nonneg(float v) { return isfinite(v) && v >= 0.0f; }

static void set_gains(pid_gains_t *g, float kp, float ki, float kd, float i_limit, float out_limit, float d_lpf)
{
    g->kp = kp; g->ki = ki; g->kd = kd;
    g->i_limit = i_limit; g->out_limit = out_limit; g->d_lpf_hz = d_lpf;
}

void params_set_defaults(params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->version = 1;

    /* ---- 2段PID（仮・小さめ）----
     * 外側 [(rad/s)/rad]: roll/pitch 4 → 10°(0.17rad)のずれで 0.7rad/s を要求。上限 3rad/s(約170°/s)。
     * 内側 [g/(rad/s)]: 1rad/s のずれで 0.8g の差。上限 6g（最大約39gの15%）、I項の上限はその約17%。 */
    for (int i = 0; i < 2; i++) {
        set_gains(&p->pid.angle[i], 4.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f);
        set_gains(&p->pid.rate[i],  0.8f, 1.0f, 0.0f, 1.0f, 6.0f, 50.0f);
    }
    set_gains(&p->pid.angle[2], 2.0f, 0.0f, 0.0f, 0.0f, 1.5f, 0.0f);
    set_gains(&p->pid.rate[2],  0.5f, 0.5f, 0.0f, 0.5f, 3.0f, 50.0f);
    p->pid.i_enable_thrust_g = 3.0f;   /* 仮: 回り始めの手前。これ以下では積分しない */

    /* ---- 姿勢推定（仮）---- */
    p->attitude.tau_s              = 0.5f;    /* 仮: 実機で調整 */
    p->attitude.still_gyro_std_rps = 0.02f;   /* 仮: 約1.1dps。ODR 7.68kHz の8組平均の雑音（約0.06dps）より十分大きい */
    p->attitude.still_gyro_abs_max_rps = 0.1f; /* 仮・提案: 約5.7dps。センサーのゼロ点ずれ(数dps以下)より大きく取る */
    p->attitude.still_accel_tol_g  = 0.05f;
    p->attitude.still_hold_ms      = 1000;
    p->attitude.still_window_s     = 0.25f;
    p->attitude.bias_tau_s         = 1.0f;

    /* ---- 機体 ---- */
    /* TODO: プロペラを付けて回す前に必ず確定。仮: M1=前左 M2=前右 M3=後右 M4=後左、M1とM3がCW。
     * 列 = {roll, pitch, yaw}。roll>0 は左側の推力を上げる、pitch>0 は前側を上げる。 */
    static const int8_t sign[FC_NUM_MOTORS][3] = {
        { +1, +1, +1 },   /* M1 前左 CW  */
        { -1, +1, -1 },   /* M2 前右 CCW */
        { -1, -1, +1 },   /* M3 後右 CW  */
        { +1, -1, -1 },   /* M4 後左 CCW */
    };
    memcpy(p->airframe.mixer_sign, sign, sizeof(sign));
    p->airframe.thrust_k_g_per_vduty   = 9.4f;   /* HW実測 */
    p->airframe.voltage_min_calc_v     = 3.0f;   /* 提案 */
    p->airframe.throttle_full_thrust_g = 28.0f;  /* 仮: 3.5Vで満タン約33g より下に余裕を残す */

    /* ---- 電池 ---- */
    p->battery.adc_divider_ratio = 2.0f;   /* 仮置き。TODO: HW設計書で確認 */
    p->battery.lpf_tau_s         = 1.0f;   /* 仮 */
    p->battery.adc_vref_v        = 3.3f;
    p->battery.v_max             = 4.2f;
    p->battery.v_min             = 3.5f;   /* 推力の直線性を実測した下限 */
    p->battery.v_low             = 3.6f;   /* 仮 */
    p->battery.low_hold_ms       = 500;    /* 仮 */

    /* ---- 安全系（③bで使う。今は既定値だけ）---- */
    p->safety.tilt_estop_rad     = 60.0f * 0.017453293f;
    p->safety.comm_timeout_ms    = 500;
    p->safety.imu_timeout_us     = 5000;
    p->safety.wdt_timeout_ms     = 100;
    p->safety.arm_level_tol_rad  = 10.0f * 0.017453293f;
    p->safety.arm_throttle_tol   = 0.05f;
    p->guard.tilt_takeover_rad   = 30.0f * 0.017453293f;
    p->guard.rate_takeover_rps   = 8.0f;
    p->guard.policy_sat_ratio    = 0.95f;
    p->guard.policy_sat_cycles   = 50;
    p->guard.policy_budget_us    = 600;
    p->descent.thrust_ratio      = 0.9f;
    p->descent.max_time_ms       = 3000;
    p->descent.shock_g           = 3.0f;
    p->descent.quiet_rate_rps    = 0.3f;
    p->descent.quiet_accel_tol_g = 0.1f;
    p->descent.quiet_hold_ms     = 500;

    /* 方策の正規化: 恒等（学習側と決めるまで）、IMU取り付け: 軸そのまま */
    for (int i = 0; i < FC_CONTROLLER_INPUT_DIM; i++) { p->policy_norm.in_scale[i] = 1.0f; }
    for (int i = 0; i < FC_CONTROLLER_OUTPUT_DIM; i++) { p->policy_norm.out_scale[i] = 1.0f; }
    /* TODO: 設計メモで確定するまで仮（恒等）。値はオーナーが導いて設計メモに書く */
    p->imu_mount.axis_map[0] = 1; p->imu_mount.axis_map[1] = 2; p->imu_mount.axis_map[2] = 3;
}

static bool gains_ok(const pid_gains_t *g)
{
    return ok_nonneg(g->kp) && ok_nonneg(g->ki) && ok_nonneg(g->kd) &&
           ok_nonneg(g->i_limit) && ok_nonneg(g->out_limit) && ok_nonneg(g->d_lpf_hz);
}

bool params_validate(const params_t *p)
{
    for (int i = 0; i < 3; i++) {
        if (!gains_ok(&p->pid.angle[i]) || !gains_ok(&p->pid.rate[i])) return false;
        /* I項の上限が出力上限を超えると意味がない */
        if (p->pid.angle[i].i_limit > p->pid.angle[i].out_limit) return false;
        if (p->pid.rate[i].i_limit > p->pid.rate[i].out_limit) return false;
    }
    if (!ok_nonneg(p->pid.i_enable_thrust_g)) return false;
    const attitude_params_t *a = &p->attitude;
    if (!(isfinite(a->tau_s) && a->tau_s > 0.0f) || !ok_nonneg(a->still_gyro_std_rps) ||
        !ok_nonneg(a->still_accel_tol_g) || !ok_nonneg(a->still_gyro_abs_max_rps) || !(a->still_window_s > 0.0f) || !(a->bias_tau_s > 0.0f)) return false;
    const airframe_params_t *af = &p->airframe;
    if (!mixer_sign_table_valid(af->mixer_sign)) return false;
    if (!(isfinite(af->thrust_k_g_per_vduty) && af->thrust_k_g_per_vduty > 0.0f)) return false;
    if (!(isfinite(af->voltage_min_calc_v) && af->voltage_min_calc_v > 0.0f)) return false;
    if (!ok_nonneg(af->throttle_full_thrust_g)) return false;
    if (!imu_mount_valid(&p->imu_mount)) return false;
    const battery_params_t *b = &p->battery;
    if (!(b->adc_divider_ratio > 0.0f) || !(b->lpf_tau_s > 0.0f) || !(b->v_min < b->v_max) ||
        !(b->v_low >= b->v_min) || !(b->v_low <= b->v_max)) return false;
    return true;
}

bool params_set_by_id(params_t *p, uint16_t id, float value)
{
    if (!isfinite(value)) return false;
    params_t tmp = *p;
    switch (id) {
    case PARAM_ID_PID_ANGLE_KP_ROLL: tmp.pid.angle[0].kp = value; break;
    case PARAM_ID_PID_RATE_KP_ROLL:  tmp.pid.rate[0].kp = value; break;
    case PARAM_ID_TILT_TAKEOVER_RAD: tmp.guard.tilt_takeover_rad = value; break;
    case PARAM_ID_TILT_ESTOP_RAD:    tmp.safety.tilt_estop_rad = value; break;
    case PARAM_ID_COMM_TIMEOUT_MS:
        if (value < 0.0f) return false;
        tmp.safety.comm_timeout_ms = (uint32_t)value; break;
    case PARAM_ID_BATT_V_LOW:        tmp.battery.v_low = value; break;
    default: return false;
    }
    if (!params_validate(&tmp)) return false;
    *p = tmp;
    return true;
}

const params_t *params_get(void) { return &s_params; }
params_t *params_get_mutable(void) { return &s_params; }
