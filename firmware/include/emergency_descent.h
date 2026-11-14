/**
 * @file emergency_descent.h
 * @brief 非常時降下の目標生成と着地検出（設計メモ5章）。制御器は必ずPID。
 *
 * コア0、ループの中。スロットルは直前のホバリングより少し下で保ち、
 * 着地は「衝撃ピークのあと、角速度≈0・加速度≈1gの静かな状態が一定時間」で検出する。
 * 数値は飛行試験で決める（params.h descent_params_t）。
 */
#ifndef EMERGENCY_DESCENT_H
#define EMERGENCY_DESCENT_H

#include <stdint.h>
#include <stdbool.h>
#include "fc_types.h"
#include "params.h"
#include "attitude.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 直前のホバリング推力を追うローパスの係数（1周期あたり）。約940Hzで時定数約0.5秒（提案、仮）。 */
#define DESCENT_HOVER_LPF_ALPHA 0.002f

typedef struct {
    float    hover_thrust_est_g; /**< 飛行中のスロットルのゆっくりした平均 */
    bool     hover_primed;       /**< 内部: 最初の1回は平均をそのサンプルで初期化 */
    uint64_t start_us;
    bool     shock_seen;
    bool     quiet_active;       /**< 内部: 衝撃のあと静かな状態が続いている最中 */
    uint64_t quiet_since_us;
    float    yaw_hold_rad;       /**< 降下開始時のヨー（保持する目標） */
} descent_t;

typedef struct {
    euler_t target_attitude;  /**< 提案: ロール/ピッチ0、ヨーは降下開始時の値 */
    float   target_thrust_g;
    bool    landed;
    bool    timed_out;
} descent_target_t;

void descent_init(descent_t *d);

/** 飛行中の推力指令を取り込んで「直前のホバリング推力」を更新。コア0、FLIGHT中に毎周期。 */
void descent_track_hover(descent_t *d, float thrust_cmd_g);

/** 降下開始。コア0、FS_EMERGENCY_DESCENT に入った周期に1回。 */
void descent_begin(descent_t *d, const attitude_state_t *att, uint64_t now_us);

/** 降下中の目標と着地・時間切れを返す。コア0、降下中に毎周期。 */
void descent_update(descent_t *d, const descent_params_t *p, const attitude_state_t *att,
                    uint64_t now_us, descent_target_t *out);

#ifdef __cplusplus
}
#endif

#endif /* EMERGENCY_DESCENT_H */
