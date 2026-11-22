/**
 * @file fc_types.h
 * @brief 全モジュール共通の基本型と定数。SDK非依存。
 *
 * 単位の約束（角度・角速度・加速度・時間はメモ未記載の提案。推力・3軸指令は2026-11-08にオーナー決定）:
 *   角度 rad / 角速度 rad/s / 加速度 g / 時間 µs
 *   推力・ロール/ピッチ/ヨー指令は すべて「1モーターあたりの推力 [g]（グラム重）」でそろえる
 *   （2026-11-08 オーナー決定。理由は docs/step3a_control_chain.md）
 */
#ifndef FC_TYPES_H
#define FC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** モーター数（M1..M4）。配列の添字は 0..3 = M1..M4。 */
#define FC_NUM_MOTORS 4

/** 制御器の入力数。姿勢3+角速度3+目標姿勢3+目標推力1+前回出力4（設計メモ1章）。 */
#define FC_CONTROLLER_INPUT_DIM 14

/** 制御器の出力数。推力1+3軸トルク（設計メモ1章）。 */
#define FC_CONTROLLER_OUTPUT_DIM 4

/** 3次元ベクトル（機体座標系 x前・y左・z上（右手系、2026-11-08確定））。 */
typedef struct {
    float x;
    float y;
    float z;
} vec3f_t;

/** オイラー角 [rad]。 */
typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} euler_t;

#ifdef __cplusplus
}
#endif

#endif /* FC_TYPES_H */
