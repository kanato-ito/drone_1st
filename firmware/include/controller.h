/**
 * @file controller.h
 * @brief 制御器の共通インターフェース。PIDと強化学習の方策を差し替えられる形。
 *
 * 入力14 = 姿勢3+角速度3+目標姿勢3+目標推力1+前回出力4、出力4 = 推力+3軸トルク
 * （設計メモ1章）。コア0、制御ループの中で1周期に1回。
 * 単位は決定（2026-11-08）: 推力・ロール/ピッチ/ヨー指令はすべて「1モーターあたりの推力 [g]」。
 *   thrust_g = 4モーターの平均推力、torque_g[i] = その軸のモーター間の推力差の片側ぶん（ミキサーが±で配る）。
 *   アーム長やヨー係数を今は持ち込まず、PIDのゲインを実機で合わせれば済むようにするため。
 * 前回出力は「実際に指令した（上限処理後の）値」。
 * 差し替えは関数ポインタの表(ops)で行う。他案: 条件分岐(switch)は制御器が増えると
 * 呼び出し側が膨らむので採らなかった。
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "fc_types.h"
#include "attitude.h"

typedef enum {
    CONTROLLER_PID = 0,
    CONTROLLER_POLICY = 1
} controller_id_t;

typedef struct {
    float thrust_g;       /**< 1モーターあたりの推力（4つの平均）[g] */
    float torque_g[3];    /**< roll, pitch, yaw の指令 [g]（モーター間の推力差の片側ぶん） */
} controller_output_t;

typedef struct {
    euler_t attitude;               /**< 姿勢角 */
    vec3f_t rate_rps;               /**< 角速度 */
    euler_t target_attitude;        /**< 目標姿勢 */
    float   target_thrust_g;        /**< 目標推力（1モーターあたり）[g] */
    controller_output_t prev_output;/**< 前回出力 */
} controller_input_t;

/**
 * 姿勢推定の結果と目標から制御器の入力を作る。
 * ヨーは「目標とのずれ」を ±π に折り返して渡す（rev22）: attitude.yaw = wrap(yaw - 目標yaw)、
 * target.yaw = 0。こうすると ±180° をまたいでも制御器が逆回りに大回りしない。
 * 呼ぶコア: コア0。頻度: 1周期に1回。
 */
void controller_input_make(const attitude_state_t *att, const euler_t *target, float target_thrust_g,
                           const controller_output_t *prev, controller_input_t *in);

/**
 * 入力を方策用の平らな float[14] に詰める。順序（学習側と一致させる）:
 * [0..2]姿勢(roll,pitch,yaw) [3..5]角速度(x,y,z) [6..8]目標姿勢 [9]目標推力
 * [10]前回推力 [11..13]前回トルク(roll,pitch,yaw)
 * 呼ぶコア: コア0。頻度: 方策使用時に1周期に1回。
 */
void controller_input_pack(const controller_input_t *in, float out[FC_CONTROLLER_INPUT_DIM]);

typedef struct {
    const char *name;
    controller_id_t id;
    /** 内部状態を初期化（積分など）。飛行指示待機での選択時と、アーム時に呼ぶ。 */
    void (*reset)(void *self);
    /** 1ステップ計算。dt_s は周期。失敗(重み未ロード等)で false。 */
    bool (*step)(void *self, const controller_input_t *in, float dt_s, controller_output_t *out);
} controller_ops_t;

/** 制御器のハンドル（ops と実体の組）。静的に確保した実体を指す。 */
typedef struct {
    const controller_ops_t *ops;
    void *self;
} controller_t;

#endif /* CONTROLLER_H */
