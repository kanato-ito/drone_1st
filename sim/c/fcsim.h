/**
 * @file fcsim.h
 * @brief シミュレータとファームの境目。Python（ctypes）から呼ぶ関数だけをここに並べる。
 *
 * 中身は3つ:
 *  1. ポートの偽物（port_sim.c）を外から操作する関数 … IMUのサンプルを入れる、時刻を進める、
 *     電池ADCの値を決める、PWMのデューティを読む。偽物が守ることは docs/hw_boundary.md 3章。
 *  2. コア1の役 … 操縦パケットを本物のメールボックス（mailbox_write）に書き、旗（ipc_flags）を立てる。
 *  3. 1周期を回す関数（fcsim_step）… 本物の control_loop_run_once を1回呼ぶ（ステップ③b以降）。
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

/* 試験用: INT1ピンを High のままにする（張り付き）。0 で戻す。 */
FCSIM_API void fcsim_set_int1_pin_high(int32_t high);

/* 試験用: 次の n 回の imu_read_fifo を SPIの失敗（IMU_ERR_SPI）にする。FIFOの中身は残る。 */
FCSIM_API void fcsim_imu_fail_reads(uint32_t n);

/** fc_wdt_start に渡された時間 [ms]（0 = まだ始まっていない）。 */
FCSIM_API uint32_t fcsim_wdt_timeout_ms(void);

/* ---------------- 2. コア1の役 ---------------- */

/**
 * 操縦パケットを1つ受けたことにする（コア1の役）。本物の mailbox_write で書き、
 * 旗に CTRL_FLAG_DISARM / CTRL_FLAG_ESTOP があれば ipc_flags_request_disarm / _estop も呼ぶ。
 * roll/pitch/yaw は rad、throttle は 0..1（ファームが × throttle_full_thrust_g で推力にする）。
 * flags は control_packet.h の CTRL_FLAG_*。実機ではおよそ50Hzで届く。
 */
FCSIM_API void fcsim_send_packet(float roll, float pitch, float yaw, float throttle, uint32_t flags);

/* ---------------- 3. ファームを回す ---------------- */

/** 1周期ごとの記録（グラフ用）。並びを変えたら Python の fc.py も直すこと。 */
typedef struct {
    int32_t  state;           /**< flight_state_t（0 起動 / 1 通信待機 / 2 飛行指示待機 / 3 飛行 / 4 非常時降下 / 5 非常停止） */
    int32_t  estop_reason;    /**< estop_reason_t */
    int32_t  descent_reason;  /**< descent_reason_t */
    int32_t  armed;           /**< 1 = 飛行か非常時降下（モーターを回してよい状態） */
    int32_t  tripped;         /**< 1 = 非常停止 */
    int32_t  motors_on;       /**< 1 = この周期 motor_pwm_set を呼んだ */
    int32_t  still;           /**< 姿勢推定の静止判定 */
    int32_t  bias_ready;      /**< バイアスを求め終わった */
    int32_t  batch_count;     /**< この周期に読んだサンプル数 */
    int32_t  batt_low;        /**< 電池低下を確定した */
    int32_t  int1_stuck;      /**< INT1張り付きをラッチした */
    int32_t  int1_pending;    /**< 読み終えてもINT1ピンがHigh（すぐもう一度回す） */
    uint32_t no_sample_count; /**< サンプル0個の連続周期数 */
    uint32_t imu_fail_count;  /**< 読み出しの連続失敗数 */
    uint32_t cycle;           /**< 処理した周期の数（stats.cycles） */
    float    angle_rad[3];    /**< 推定した姿勢 roll, pitch, yaw */
    float    rate_rps[3];     /**< 推定した角速度（バイアス補正後） */
    float    accel_g[3];      /**< 直近の加速度 */
    float    bias_rps[3];     /**< ジャイロのバイアス推定 */
    float    target_rad[3];   /**< 目標姿勢 roll, pitch, yaw（ヨーはアーム時の向きを基準に合わせたもの） */
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
    float    duty[4];         /**< PWMに出したデューティ（ポートの偽物が受けた値） */
} fcsim_telemetry_t;

/**
 * 起動処理（params を既定値に → 電池ADCを batt_v に → ipc の旗とメールボックスを空に → control_loop_init）。
 * 実機の電源投入と同じく、ADCには最初から電池の電圧が出ている（ローパスの初期値になる）。
 * 0 = 成功。何度呼んでも最初からやり直す。起動に失敗すると状態が非常停止になる（戻り値は0のまま）。
 */
FCSIM_API int32_t fcsim_init(float batt_v);

/**
 * 1周期回す（control_loop_run_once を1回）。呼ぶ前に fcsim_set_time_us で時計を合わせておく。
 * int1 が 1 なら INT1 の立ち上がりが来たことにする（時刻は今の時計）。0 なら来ない周期
 * （IMU途絶。run_once(NULL)）。FIFOが空でも int1=1 なら「来たのにサンプル0個」の周期になる。
 * 戻り値: run_once の戻り値（1 = 読み終えてもINT1ピンがHigh。呼び出し側はすぐもう一度回す）。
 */
FCSIM_API int32_t fcsim_step(int32_t int1);

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
