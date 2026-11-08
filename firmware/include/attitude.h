/**
 * @file attitude.h
 * @brief 姿勢推定。制御器の外に置き、PIDと方策と見張り役が同じ姿勢を見る（設計メモ1章）。SDK非依存。
 *
 * 設計メモ rev22 で決定した方式:
 *  - ロール・ピッチ: 相補フィルタ（短い時間はジャイロ、長い時間は加速度。時定数τは params）
 *  - ヨー: ジャイロの積分だけ（磁気なし。ゆっくりずれるのは受け入れる）
 *  - ジャイロのバイアス: 静止と判定している間だけ平均を更新し、アーム時に固定する
 *
 * 座標: IMU基板の軸のまま（取り付け補正は未実装）。右手系、z上向きで水平なら加速度は (0,0,+1g)。
 *   roll = x軸まわり、pitch = y軸まわり、yaw = z軸まわり。Z-Y-X オイラー角。
 * コア0、制御ループの中。1周期に1回 attitude_update_batch を呼ぶ。
 */
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>
#include <stdbool.h>
#include "fc_types.h"
#include "params.h"
#include "imu_lsm6dsv16x.h"

/** 制御器・見張り役・着地検出が読む姿勢の出力。 */
typedef struct {
    euler_t angle;      /**< 姿勢角 [rad]。ヨーは ±π に折り返し済み */
    vec3f_t rate_rps;   /**< 機体の角速度 [rad/s]（バイアス補正済み） */
    vec3f_t accel_g;    /**< 直近の加速度 [g]（着地検出用） */
    bool    valid;      /**< 初期化済みか */
} attitude_state_t;

typedef struct {
    attitude_state_t out;
    const attitude_params_t *params;
    vec3f_t bias_rps;       /**< ジャイロのバイアス推定 [rad/s] */
    bool    bias_locked;    /**< true の間はバイアスを更新しない（アーム中） */
    bool    bias_ready;     /**< 静止確定でバイアスを一度でも求めた */
    bool    still;          /**< 静止と確定している */
    float   still_time_s;   /**< 静止条件が連続で満たされている時間 */
    vec3f_t gyro_mean;      /**< 内部: 窓平均（バッチ平均のEMA） */
    vec3f_t gyro_var;       /**< 内部: ばらつき（分散）のEMA */
    bool    stat_primed;    /**< 内部: 平均の初期化済み */
} attitude_estimator_t;

/** 初期化。params は参照を保持する（変更は待機中のみなので参照でよい）。コア0、起動時に1回。 */
void attitude_init(attitude_estimator_t *e, const attitude_params_t *params);

/** 加速度から roll/pitch を直接合わせ、ヨーを0にする。コア0、起動直後や待機中に使ってよい。 */
void attitude_align_from_accel(attitude_estimator_t *e, const imu_sample_t *s);

/**
 * 1周期ぶんのサンプルを取り込んで姿勢を更新する。
 * 呼ぶコア: コア0。頻度: 1周期に1回（約940Hz）。dt_s はサンプル間隔 imu_sample_period_s()。
 * 最初の1回は加速度から姿勢を合わせる。バッチ平均で更新する（トリグ関数をバッチあたり1回に抑えるため）。
 */
void attitude_update_batch(attitude_estimator_t *e, const imu_batch_t *batch, float dt_s);

/** アーム時 true でバイアスを固定、待機に戻ったら false。コア0。 */
void attitude_set_bias_locked(attitude_estimator_t *e, bool locked);

/** 静止と確定しているか（ジャイロのばらつき小・|a|≈1g が still_hold_ms 続いた）。 */
bool attitude_is_still(const attitude_estimator_t *e);

/** 静止確定でバイアスを一度でも求めたか。アーム可否の条件に使う。 */
bool attitude_bias_ready(const attitude_estimator_t *e);

/** 現在の姿勢（読み取り専用）。コア0、随時。 */
const attitude_state_t *attitude_get(const attitude_estimator_t *e);

/** 機体の鉛直からの傾き [rad]（ロール/ピッチ個別でなく合成）。見張り役(30°)、非常停止(60°)、水平確認に使う。 */
float attitude_tilt_rad(const attitude_state_t *s);

/** 角度を (-π, π] に折り返す。ヨーの「目標とのずれ」を渡すときに使う（rev22）。 */
float attitude_wrap_pi(float rad);

#endif /* ATTITUDE_H */
