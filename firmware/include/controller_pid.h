/**
 * @file controller_pid.h
 * @brief 2段PID制御器（rev22）。3軸とも 外側: 角度の誤差→目標角速度、内側: 角速度の誤差→指令[g]。
 *
 * 非常時降下では必ずこれを使う。コア0、ループの中。見張り役のため、方策使用中も
 * 「影」として毎周期回す案を提案（architecture.md #13）。
 * 積分: reset() で0。目標推力が i_enable_thrust_g を超えている間だけためる。上限は i_limit（I項の寄与の上限）。
 * 単位: 外側 kp [(rad/s)/rad]、内側 kp [g/(rad/s)]、内側の出力 = torque_g。
 */
#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

#include <stdbool.h>
#include "controller.h"
#include "params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float integ;       /**< I項の出力への寄与（ki 込み） */
    float prev_meas;
    float d_filt;
    bool  primed;      /**< prev_meas が有効か（最初の微分を捨てる） */
} pid_axis_state_t;

typedef struct {
    const pid_params_t *params;   /**< 変更は飛行指示待機のみなので参照でよい */
    pid_axis_state_t angle[3];
    pid_axis_state_t rate[3];
    float last_rate_target_rps[3]; /**< 表示・ログ用: 外側の出力 */
} pid_controller_t;

/** 初期化。コア0、起動時に1回。 */
void controller_pid_init(pid_controller_t *c, const pid_params_t *params);

/** 共通インターフェースのハンドルを返す。コア0、起動時に1回。 */
controller_t controller_pid_handle(pid_controller_t *c);

/**
 * PID 1段ぶん（試験用に公開）。err = 目標-測定、meas は微分用の測定値（測定値の微分を使い目標変更で跳ねない）。
 * integ_enable が false の間は積分をためない（保持する）。出力は ±out_limit に制限。
 */
float pid_axis_update(pid_axis_state_t *st, const pid_gains_t *g, float err, float meas,
                      float dt_s, bool integ_enable);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_PID_H */
