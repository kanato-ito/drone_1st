/**
 * @file mixer.h
 * @brief ミキサー: 全体推力+3軸指令（すべて g）→ 4モーターの推力 [g]（rev22）。SDK非依存の純粋関数。
 *
 * 各モーターの推力 = 全体 + s_roll*ロール + s_pitch*ピッチ + s_yaw*ヨー（符号は params の表 1か所）。
 * 出力は 0 〜 motor_max_g に必ず収める。はみ出したときの優先（ロール・ピッチ最優先）:
 *   1. ヨーを削る（収まる最大の割合まで。必要なら0まで）
 *   2. 4つを同じだけ上下にずらす（全体推力を犠牲にする）
 *   3. モーター同士の差が motor_max_g より大きくて収まらないときだけ、ロール・ピッチを縮める
 * デューティへの換算は thrust_model.h（ミキサーを電池・推力特性から独立させるため別ファイル）。
 */
#ifndef MIXER_H
#define MIXER_H

#include <stdbool.h>
#include "fc_types.h"
#include "controller.h"
#include "params.h"

typedef struct {
    float motor_g[FC_NUM_MOTORS]; /**< 各モーターの推力 [g]、0..motor_max_g */
    bool  saturated;              /**< 以下のどれかが働いた（ログ用） */
    float yaw_scale;              /**< 1=ヨーそのまま、0=全部削った */
    float shift_g;                /**< 4つ同じだけずらした量 [g]（負=下げた） */
    float rp_scale;               /**< 1=ロール・ピッチそのまま、<1=縮めた */
} mixer_result_t;

/** 符号表が正しいX字か検査: 各要素が±1、3列とも和0、列同士が直交（対角が同じ回転方向）。params_validate でも使う。 */
bool mixer_sign_table_valid(const int8_t sign[FC_NUM_MOTORS][3]);

/**
 * 制御器出力をモーターごとの推力に配分する。
 * 呼ぶコア: コア0。頻度: 1周期に1回、制御器の直後。NaN/inf が入ったら全部0にする。
 * @param motor_max_g  1モーターが出せる最大推力 [g]（thrust_model_max_g(電池電圧)）
 */
void mixer_mix(const airframe_params_t *af, const controller_output_t *u,
               float motor_max_g, mixer_result_t *out);

#endif /* MIXER_H */
