/**
 * @file mixer.h
 * @brief ミキサー: 推力+3軸トルク → 4モーターのデューティ。
 *
 * コア0、ループの中で1周期に1回。純粋関数にしておく（PCで試験しやすい）。
 * 配置・推力特性・電圧範囲は params.h の airframe/battery。
 * 飽和したときの優先順位と電圧補償の有無はメモに無く未決（architecture.md）。
 */
#ifndef MIXER_H
#define MIXER_H

#include <stdbool.h>
#include "fc_types.h"
#include "controller.h"
#include "params.h"

typedef struct {
    float duty[FC_NUM_MOTORS];   /**< 0..1 */
    bool  saturated;             /**< どれかが0または1に張りついた（ログ用） */
} mixer_result_t;

/**
 * 制御器出力をモーターごとの推力に配分し、推力特性の逆引きでデューティにする。
 * 呼ぶコア: コア0。頻度: 1周期に1回、制御器の直後。
 * @param battery_v  現在の電池電圧（電圧補償用。補償しない場合は無視）
 */
void mixer_mix(const airframe_params_t *af, const controller_output_t *u,
               float battery_v, mixer_result_t *out);

#endif /* MIXER_H */
