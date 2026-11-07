/**
 * @file attitude.h
 * @brief 姿勢推定。制御器の外に置き、PIDと方策と見張り役が同じ姿勢を見る（設計メモ1章）。
 *
 * コア0、制御ループの中。アルゴリズム（相補/Mahony等）はメモに無く未決。
 * 内部状態はアルゴリズム次第で変わる（静的確保のためここで定義している）。
 */
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>
#include <stdbool.h>
#include "fc_types.h"
#include "imu_lsm6dsv16x.h"

/** 制御器・見張り役・着地検出が読む姿勢の出力。 */
typedef struct {
    euler_t angle;      /**< 姿勢角 [rad] */
    vec3f_t rate_rps;   /**< 角速度 [rad/s] */
    vec3f_t accel_g;    /**< 直近の加速度 [g]（着地検出用） */
    bool    valid;      /**< 初期化済みか */
} attitude_state_t;

typedef struct {
    attitude_state_t out;
    float q[4];         /**< 内部: クォータニオン（提案。実装で変えてよい） */
    float integral[3];  /**< 内部: 補正の積分項など */
} attitude_estimator_t;

/** 初期化。コア0、起動時に1回。 */
void attitude_init(attitude_estimator_t *e);

/** 停止中の加速度から初期姿勢を合わせる。コア0、飛行指示待機の間に繰り返し呼んでよい。 */
void attitude_align_from_accel(attitude_estimator_t *e, const imu_sample_t *s);

/**
 * 1周期ぶんのサンプルを順に取り込んで姿勢を更新する。
 * 呼ぶコア: コア0。頻度: 1周期に1回（約940Hz、内部で batch->count 回まわす）。
 * dt_s は imu_sample_period_s()。
 */
void attitude_update_batch(attitude_estimator_t *e, const imu_batch_t *batch, float dt_s);

/** 現在の姿勢（読み取り専用）。コア0、随時。 */
const attitude_state_t *attitude_get(const attitude_estimator_t *e);

/** 機体の鉛直からの傾き [rad]（ロール/ピッチ個別でなく合成）。見張り役(30°)、非常停止(60°)、水平確認に使う。 */
float attitude_tilt_rad(const attitude_state_t *s);

#endif /* ATTITUDE_H */
