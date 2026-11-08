/**
 * @file fcsim.h
 * @brief シミュレータとファームの境目。Python（ctypes）から呼ぶ関数だけをここに並べる。
 *
 * 中身は2つ:
 *  1. ポートの偽物（port_sim.c）を外から操作する関数 … IMUのサンプルを入れる、時刻を進める、
 *     電池ADCの値を決める、PWMのデューティを読む。偽物が守ることは docs/hw_boundary.md 3章。
 *  2. 1周期を回す関数（fcsim_step）… いまは control_bench.c と同じ順に③aの信号の流れを呼ぶ
 *     「仮の1周期」。ファームに control_loop_run_once（ステップ③b）ができたら、中身をそれに
 *     置き換える。Python側の呼び方は変えない。
 *
 * ファームの状態（params など）は静的変数1組だけなので、1プロセスで同時に動かせる機体は1機。
 * 強化学習で並列に回すときはプロセスを分ける。
 */
#ifndef FCSIM_H
#define FCSIM_H

#include <stdint.h>

#if defined(_WIN32)
#define FCSIM_API __declspec(dllexport)
#else
#define FCSIM_API __attribute__((visibility("default")))
#endif

/* ---------------- 1. ポートの偽物の操作 ---------------- */

/** IMUのFIFOに1サンプル（同時刻のジャイロ+加速度）を積む。単位は rad/s と g、軸はIMU基板の軸。 */
FCSIM_API void fcsim_imu_push(const float gyro_rps[3], const float accel_g[3]);

/** FIFOにたまっているサンプル数。 */
FCSIM_API int32_t fcsim_imu_pending(void);

/** 偽物が返すサンプル間隔 [s]（imu_sample_period_s と同じ値。物理の刻みに使う）。 */
FCSIM_API float fcsim_imu_sample_period_s(void);

/** 偽物の時計を進める [µs]。単調増加であること（戻そうとしたら無視する）。 */
FCSIM_API void fcsim_set_time_us(uint64_t now_us);

/** 電池ADCの生の値（12bit）を直接決める。 */
FCSIM_API void fcsim_set_batt_raw(uint16_t raw);

/** 電池の端子電圧 [V] を、ファームの params（分圧比・基準電圧）で12bitの生の値に直して決める。 */
FCSIM_API void fcsim_set_batt_voltage(float volts);

/** motor_pwm_set で最後に受けたデューティ（0..1、M1..M4）。 */
FCSIM_API void fcsim_get_duty(float duty_out[4]);

/** fc_wdt_feed が呼ばれた回数。 */
FCSIM_API uint32_t fcsim_wdt_feed_count(void);

/* ---------------- 2. ファームを回す ---------------- */

/** 1周期ごとの記録（グラフ用）。並びを変えたら Python の fc.py も直すこと。 */
typedef struct {
    int32_t  armed;           /**< 1 = 制御を回している */
    int32_t  tripped;         /**< 1 = 傾き過大で止めた（ディスアームまで保持） */
    int32_t  still;           /**< 姿勢推定の静止判定 */
    int32_t  bias_ready;      /**< バイアスを求め終わった */
    int32_t  batch_count;     /**< この周期に読んだサンプル数 */
    uint32_t cycle;           /**< 周期番号 */
    float    angle_rad[3];    /**< 推定した姿勢 roll, pitch, yaw */
    float    rate_rps[3];     /**< 推定した角速度（バイアス補正後） */
    float    accel_g[3];      /**< 直近の加速度 */
    float    bias_rps[3];     /**< ジャイロのバイアス推定 */
    float    target_rad[3];   /**< 目標姿勢 roll, pitch, yaw */
    float    target_thrust_g; /**< 目標推力（1モーターあたり） */
    float    rate_target_rps[3]; /**< 外側PIDの出力（目標角速度） */
    float    out_thrust_g;    /**< 制御器の出力: 推力 */
    float    out_torque_g[3]; /**< 制御器の出力: roll, pitch, yaw 指令 */
    float    rate_integ_g[3]; /**< 内側PIDの積分（I項の寄与） */
    float    motor_g[4];      /**< ミキサーの出力: 各モーターの推力 */
    int32_t  mix_saturated;
    float    mix_yaw_scale;
    float    mix_shift_g;
    float    mix_rp_scale;
    float    batt_v;          /**< ローパス後の電池電圧（ファームが換算に使う値） */
    float    batt_raw_v;      /**< ローパス前 */
    float    duty[4];         /**< 出したデューティ */
} fcsim_telemetry_t;

/** 起動処理（params を既定値に、IMU・PWM・電池・姿勢推定・PIDを初期化）。0 = 成功。何度呼んでも最初からやり直す。 */
FCSIM_API int32_t fcsim_init(void);

/** 目標（操縦パケットの代わり）。角度は rad、推力は1モーターあたりの g。ヨーはアーム時の向きからのずれ。 */
FCSIM_API void fcsim_set_target(float roll_rad, float pitch_rad, float yaw_rad, float thrust_g);

/**
 * アーム要求（仮の簡易手順。状態機械はステップ③bでファームに入る）。
 * 1 の間: 静止・バイアス確定・水平（params の arm_level_tol_rad 以内）がそろったらアームする。
 * 0 にするとディスアーム（モーター停止、バイアスの固定を外す）。
 */
FCSIM_API void fcsim_set_arm_request(int32_t arm);

/**
 * 1周期回す。INT1が来た時刻 [µs] を渡す（仮想時間。呼ぶ前に fcsim_set_time_us で時計を合わせておく）。
 * FIFOにたまっている分を全部読み、姿勢推定→（アーム中なら）制御器→ミキサー→推力換算→PWM。
 * 戻り値: 読んだサンプル数（0ならFIFOが空で何もしていない）。
 */
FCSIM_API int32_t fcsim_step(uint32_t int1_timestamp_us);

/** 直近の周期の記録をコピーする。 */
FCSIM_API void fcsim_get_telemetry(fcsim_telemetry_t *out);

/** sizeof(fcsim_telemetry_t)。Python側の構造体と食い違っていないかの確認用。 */
FCSIM_API int32_t fcsim_telemetry_size(void);

/* ---------------- パラメータ（試験で変える分だけ） ---------------- */

/** PIDの1軸のゲイン。stage: 0=外側(angle) 1=内側(rate)、axis: 0..2 = roll,pitch,yaw。
 *  g = {kp, ki, kd, i_limit, out_limit, d_lpf_hz}。params_validate に通らなければ変えずに -1。 */
FCSIM_API int32_t fcsim_set_pid_gains(int32_t stage, int32_t axis, const float g[6]);
FCSIM_API int32_t fcsim_get_pid_gains(int32_t stage, int32_t axis, float g_out[6]);

/** ミキサーの符号表（行 M1..M4 × 列 roll,pitch,yaw、±1）。正しいX字でなければ変えずに -1。 */
FCSIM_API int32_t fcsim_set_mixer_sign(const int8_t sign[12]);
FCSIM_API void    fcsim_get_mixer_sign(int8_t sign_out[12]);

/** 名前つきの float パラメータ（下の表にあるものだけ）。未知の名前・範囲外は -1。 */
FCSIM_API int32_t fcsim_set_param(const char *name, float value);
FCSIM_API int32_t fcsim_get_param(const char *name, float *value_out);

#endif /* FCSIM_H */
