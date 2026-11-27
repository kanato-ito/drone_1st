/**
 * @file gyro_filter.h
 * @brief ジャイロのローパス（2次バターワース、双2次 = biquad）。角速度ループの入力用。SDK非依存の純粋C。
 *
 * HW設計書・開発計画書の「2次 80Hz」（params の gyro_filter.cutoff_hz。2026-11-22 オーナー決定）。
 * 1サンプルごと（約7.5kHz）に3軸ぶん回す。形は直接形II転置（Direct Form II Transposed）、float。
 * 係数は Q=1/√2 のバターワースを、プリワープつきの双一次変換（bilinear）で作る。
 * 姿勢推定には使わない（生の値のまま）。角速度ループ（PIDと方策）の入力だけに使う。
 * コア0、制御ループの中で1周期に1回 gyro_filter_apply_batch を呼ぶ（8サンプル×3軸）。
 */
#ifndef GYRO_FILTER_H
#define GYRO_FILTER_H

#include <stdbool.h>
#include "fc_types.h"
#include "imu_lsm6dsv16x.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 1軸ぶんの状態は z1,z2。係数は3軸で共通。 */
typedef struct {
    float b0, b1, b2, a1, a2;   /**< 係数（a0=1 に正規化済み） */
    bool  enabled;              /**< false なら素通し */
    bool  seeded;               /**< 最初のサンプルで状態を初期化した */
    float z1[3], z2[3];         /**< 状態（x,y,z） */
    vec3f_t out;                /**< 最後のサンプルの出力 */
} gyro_filter_t;

/**
 * 係数を作って状態を空にする。cutoff_hz <= 0 は「無し」（素通し）。
 * sample_hz <= 0、NaN、cutoff_hz >= 0.45 × sample_hz のような不正な組も安全側で素通しにする
 * （丸めて使うことはしない。範囲は params_validate が弾く）。
 * 呼ぶコア: コア0、起動時に1回。
 */
void gyro_filter_init(gyro_filter_t *f, float cutoff_hz, float sample_hz);

/**
 * batch の全サンプル（count 個）を順に3軸ともフィルタに通し、最後のサンプルの出力を覚える。
 * batch は変えない。count==0 なら何もしない（出力は前のまま）。
 * 初期化後の最初のサンプルは、状態をその値に合わせる（起動時の過渡応答を出さない）。
 * 呼ぶコア: コア0、1周期に1回。
 */
void gyro_filter_apply_batch(gyro_filter_t *f, const imu_batch_t *batch);

/** 最後のサンプルのフィルタ出力 [rad/s]。まだ何も通していなければ 0。 */
vec3f_t gyro_filter_output(const gyro_filter_t *f);

#ifdef __cplusplus
}
#endif

#endif /* GYRO_FILTER_H */
