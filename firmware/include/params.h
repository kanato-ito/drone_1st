/**
 * @file params.h
 * @brief 機体固有の値（ゲイン、配置、推力特性、電圧範囲、しきい値）を1か所にまとめる。SDK非依存。
 *
 * 設計メモ1章。所有者はコア0だけ。コア1は直接読み書きしない（競合を避ける）。
 * 変更は飛行指示待機のときだけ、コマンドキュー経由(cmd_msg.h)でコア0が行う（設計メモ4,6章）。
 * 既定値は params.c に書く。数値は全て「仮」で、試験台・飛行試験で決める。
 * 永続化(フラッシュ保存)はメモに無く未決（architecture.md）。
 */
#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>
#include <stdbool.h>
#include "fc_types.h"

#define PARAM_THRUST_CURVE_POINTS 8

/** PIDの1軸ぶんのゲイン。 */
typedef struct {
    float kp, ki, kd;
    float i_limit;   /**< 積分の上限 */
    float out_limit; /**< 出力の上限 */
    float d_lpf_hz;  /**< 微分のローパス */
} pid_gains_t;

/** PID（提案: 姿勢角→角速度のカスケード）。添字0,1,2 = roll,pitch,yaw。 */
typedef struct {
    pid_gains_t angle[3];
    pid_gains_t rate[3];
} pid_params_t;

/** 機体の配置・推力特性（ミキサー用）。 */
typedef struct {
    float   motor_x_m[FC_NUM_MOTORS];     /**< 重心からの位置 [m]（前方+）。座標系は未決 */
    float   motor_y_m[FC_NUM_MOTORS];     /**< 同（右+） */
    int8_t  spin_dir[FC_NUM_MOTORS];      /**< 回転方向 +1/-1（反トルクの符号） */
    float   yaw_torque_per_thrust_m;      /**< 推力1Nあたりの反トルク [N·m/N] */
    float   duty_points[PARAM_THRUST_CURVE_POINTS];   /**< 推力特性: デューティ(0..1) */
    float   thrust_points_n[PARAM_THRUST_CURVE_POINTS]; /**< 同、1モーターの推力 [N]（折れ線。形は測定後に見直す） */
    float   thrust_ref_voltage_v;         /**< 上の特性を測った電圧 */
    float   max_total_thrust_n;           /**< throttle=1 のときの合計推力 [N]（スロットル→推力の換算） */
} airframe_params_t;

/** 電池。電圧範囲を含む。 */
typedef struct {
    float adc_divider_ratio; /**< 分圧比（電池電圧 = ADC電圧 × この値）。TODO: 回路図確認 */
    float adc_vref_v;        /**< ADC基準 [V] */
    float v_max;             /**< 満充電付近 [V]（電圧範囲の上端） */
    float v_min;             /**< 使用下限 [V]（電圧範囲の下端） */
    float v_low;             /**< 電池低下とみなす電圧 [V]（非常時降下のきっかけ。値は未決） */
    uint32_t low_hold_ms;    /**< v_low 未満がこの時間続いたら低下確定 */
} battery_params_t;

/** 安全系のしきい値。 */
typedef struct {
    float    tilt_estop_rad;       /**< 非常停止の傾き（仮60°） */
    uint32_t comm_timeout_ms;      /**< 通信途絶とみなす時間（未決、提案） */
    uint32_t imu_timeout_us;       /**< INT1が来ない時間（約5000） */
    uint32_t wdt_timeout_ms;       /**< ウォッチドッグ（未決、提案） */
    float    arm_level_tol_rad;    /**< アーム時の「水平」許容 */
    float    arm_throttle_tol;     /**< アーム時の「スロットル0」許容 */
} safety_params_t;

/** PIDの見張り役のしきい値（設計メモ5章）。 */
typedef struct {
    float    tilt_takeover_rad;    /**< 傾きがやや大きい（仮30°） */
    float    rate_takeover_rps;    /**< 角速度が大きすぎる */
    float    policy_sat_ratio;     /**< 出力が上限の何割以上を「張りつき」とみなすか */
    uint16_t policy_sat_cycles;    /**< 張りつきが何周期続いたら異常か */
    uint32_t policy_budget_us;     /**< 方策の計算時間の上限（周期1.04msより小さく） */
} guard_params_t;

/** 非常時降下と着地検出（数値は飛行試験で決める）。 */
typedef struct {
    float    thrust_ratio;         /**< 直前ホバリング推力に掛ける比（1より少し小さい） */
    uint32_t max_time_ms;          /**< 降下の上限時間 */
    float    shock_g;              /**< 衝撃ピークとみなす加速度 [g] */
    float    quiet_rate_rps;       /**< 静かな状態: 角速度の上限 */
    float    quiet_accel_tol_g;    /**< 静かな状態: |加速度|-1g の許容 */
    uint32_t quiet_hold_ms;        /**< 静かな状態が続く時間 */
} descent_params_t;

/** 方策の入出力の正規化（学習側と一致させる。メモ未記載、提案）。 */
typedef struct {
    float in_offset[FC_CONTROLLER_INPUT_DIM];
    float in_scale[FC_CONTROLLER_INPUT_DIM];
    float out_offset[FC_CONTROLLER_OUTPUT_DIM];
    float out_scale[FC_CONTROLLER_OUTPUT_DIM];
} policy_norm_params_t;

/** IMUの取り付け（基板座標→機体座標）。提案。 */
typedef struct {
    int8_t axis_map[3];   /**< 機体のx,y,zに使う基板の軸 (±1..±3, 符号が向き) */
} imu_mount_params_t;

/** 全パラメータ。 */
typedef struct {
    uint32_t             version;
    pid_params_t         pid;
    airframe_params_t    airframe;
    battery_params_t     battery;
    safety_params_t      safety;
    guard_params_t       guard;
    descent_params_t     descent;
    policy_norm_params_t policy_norm;
    imu_mount_params_t   imu_mount;
} params_t;

/** TCPの SET_PARAM で指定するID（提案。代表のみ、実装時に追加）。 */
typedef enum {
    PARAM_ID_PID_ANGLE_KP_ROLL = 1,
    PARAM_ID_PID_RATE_KP_ROLL  = 20,
    /* ... 他の軸・項目は実装時に追加 */
    PARAM_ID_TILT_TAKEOVER_RAD = 100,
    PARAM_ID_TILT_ESTOP_RAD    = 101,
    PARAM_ID_COMM_TIMEOUT_MS   = 102,
    PARAM_ID_BATT_V_LOW        = 120
} param_id_t;

/** 既定値（仮）を入れる。コア0、起動時に1回。 */
void params_set_defaults(params_t *p);

/** 値の範囲チェック（負のゲイン、NaN、順序の矛盾など）。コア0、起動時とパラメータ変更時。 */
bool params_validate(const params_t *p);

/** ID指定で1つ変更。範囲外・未知IDは false。コア0、飛行指示待機でのコマンド処理時のみ。 */
bool params_set_by_id(params_t *p, uint16_t id, float value);

/** 唯一のパラメータ実体への読み取りポインタ。コア0、随時。 */
const params_t *params_get(void);

/** 変更用ポインタ。コア0の起動処理とコマンド処理だけが使う。 */
params_t *params_get_mutable(void);

#endif /* PARAMS_H */
