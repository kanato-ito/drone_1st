/**
 * @file controller_pid.h
 * @brief PID制御器（姿勢角→角速度のカスケード、提案）。非常時降下では必ずこれを使う。
 *
 * コア0、ループの中。見張り役のため、方策使用中も「影」として毎周期回す案を提案
 * （取り上げ時に積分が冷えていないように。architecture.md #13）。
 */
#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

#include "controller.h"
#include "params.h"

typedef struct {
    float integ;
    float prev_meas;
    float d_filt;
} pid_axis_state_t;

typedef struct {
    const pid_params_t *params;   /**< 変更は飛行指示待機のみなので参照でよい */
    pid_axis_state_t angle[3];
    pid_axis_state_t rate[3];
} pid_controller_t;

/** 初期化。コア0、起動時に1回。 */
void controller_pid_init(pid_controller_t *c, const pid_params_t *params);

/** 共通インターフェースのハンドルを返す。コア0、起動時に1回。 */
controller_t controller_pid_handle(pid_controller_t *c);

#endif /* CONTROLLER_PID_H */
