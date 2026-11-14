/**
 * @file guard.h
 * @brief PIDの見張り役: 方策が飛ばしているとき、危険ならPIDが取り上げる（設計メモ5章）。
 *
 * コア0、ループの中。取り上げたらその飛行の間は方策に戻さない。
 * 戻せるのは飛行指示待機で選び直したときだけ（guard_reset）。
 * 判定は2段: 方策の計算前(傾き・角速度・人の指示)と、計算後(出力異常・計算時間)。
 */
#ifndef GUARD_H
#define GUARD_H

#include <stdint.h>
#include <stdbool.h>
#include "controller.h"
#include "params.h"
#include "attitude.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TAKEOVER_NONE = 0,
    TAKEOVER_TILT,            /**< 傾きがやや大きい（仮30°） */
    TAKEOVER_RATE,            /**< 角速度が大きすぎる */
    TAKEOVER_POLICY_NAN,      /**< 方策の出力がNaN/Inf */
    TAKEOVER_POLICY_SATURATED,/**< 出力が上限に張りつき */
    TAKEOVER_POLICY_OVERRUN,  /**< 方策の計算が周期内に終わらない */
    TAKEOVER_HUMAN,           /**< 人の指示（旗。パケット項目は提案） */
    TAKEOVER_POLICY_FAIL      /**< 提案: 方策の step が失敗（重み未ロードなど）を返した */
} takeover_reason_t;

typedef struct {
    takeover_reason_t reason;  /**< NONE 以外なら取り上げ済み（ラッチ） */
    uint16_t sat_count;
} guard_t;

/** 取り上げ状態を解除。コア0、飛行指示待機で制御器を選んだとき（とアーム時）のみ。 */
void guard_reset(guard_t *g);

/** 取り上げ済みか。コア0、毎周期。 */
bool guard_taken_over(const guard_t *g);

/** 理由を指定して取り上げる（すでに取り上げ済みなら何もしない）。方策の step 失敗用。コア0。 */
void guard_force_takeover(guard_t *g, takeover_reason_t reason);

/** 理由の名前（ログ・表示用）。 */
const char *takeover_reason_name(takeover_reason_t r);

/**
 * 方策を呼ぶ前の判定。コア0、方策使用中に1周期に1回。
 * 取り上げが決まれば true（この周期からPIDの出力を使う）。
 */
bool guard_check_pre(guard_t *g, const guard_params_t *gp, const attitude_state_t *att, bool human_takeover);

/**
 * 方策を呼んだ後の判定。elapsed_us は方策の計算時間。コア0、方策使用中に1周期に1回。
 * 取り上げが決まれば true（この周期の出力は影PIDの値に差し替える）。
 */
bool guard_check_post(guard_t *g, const guard_params_t *gp, const controller_output_t *policy_out,
                      float thrust_max_g, float torque_max_g, uint32_t elapsed_us);

#ifdef __cplusplus
}
#endif

#endif /* GUARD_H */
