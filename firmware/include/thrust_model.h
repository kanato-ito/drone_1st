/**
 * @file thrust_model.h
 * @brief 推力[g] ↔ デューティ比の換算（rev22）。SDK非依存の純粋関数。
 *
 * HW実測: 推力[g] ≒ K × デューティ × 電池電圧[V]（K=9.4、3.5〜4.2Vで直線）。折れ線テーブルは使わない。
 * ミキサーとは別ファイルにした: ミキサーは「配分」だけ、ここは「モーターと電池の物理」だけを持ち、
 * 試験と将来の差し替え（K の見直し、電圧補償の改良）が互いに影響しないようにするため。
 * モーターの個体差は PID の I 項で吸収する（rev22）。
 */
#ifndef THRUST_MODEL_H
#define THRUST_MODEL_H

#include "params.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 電池電圧を換算に使える範囲に丸める（下限は voltage_min_calc_v。上限は 2倍の余裕を見て非常に大きい値を弾く）。 */
float thrust_model_clamp_voltage(const airframe_params_t *af, float battery_v);

/** 1モーターの最大推力 [g] = K × 1.0 × V。ミキサーの上限。 */
float thrust_model_max_g(const airframe_params_t *af, float battery_v);

/** デューティ = 推力 ÷ (K × V)、0..1 に丸める。NaN は0。コア0、1周期に1回×4モーター。 */
float thrust_model_to_duty(const airframe_params_t *af, float thrust_g, float battery_v);

#ifdef __cplusplus
}
#endif

#endif /* THRUST_MODEL_H */
