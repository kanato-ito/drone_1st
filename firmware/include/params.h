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

/** PIDの1軸ぶんのゲイン。単位は軸ごとに異なる（controller_pid.h 参照）。 */
typedef struct {
    float kp, ki, kd;
    float i_limit;   /**< I項の出力への寄与の上限（絶対値）。初期値は out_limit の1〜2割（rev22） */
    float out_limit; /**< 出力の上限 */
    float d_lpf_hz;  /**< 微分のローパス */
} pid_gains_t;

/**
 * 2段PID（rev22）。添字0,1,2 = roll,pitch,yaw。
 * 外側 angle: 角度の誤差[rad] → 目標角速度[rad/s]。内側 rate: 角速度の誤差[rad/s] → 指令[g]。
 * 調整は内側から。内側を外側よりはっきり速くする。
 */
typedef struct {
    pid_gains_t angle[3];
    pid_gains_t rate[3];
    float i_enable_thrust_g; /**< 目標推力がこれを超えている間だけ積分をためる [g]（rev22）。仮 */
} pid_params_t;

/**
 * 相補フィルタ・ジャイロのバイアス推定・静止判定（rev22）。すべて仮。
 * 静止判定 = 「ジャイロの平均からのばらつき <= still_gyro_std_rps」かつ
 *            「| |加速度| - 1g | <= still_accel_tol_g」が still_hold_ms 続く。
 */
typedef struct {
    float    tau_s;              /**< 相補フィルタの時定数τ [s]。実機で調整。仮 */
    float    still_gyro_std_rps; /**< 静止とみなすジャイロのばらつき（標準偏差） [rad/s] */
    float    still_gyro_abs_max_rps; /**< 提案: 窓平均の大きさがこれ以上なら静止にしない [rad/s]。一定の速さで回り続けても「ばらつき」は0になるため、その誤判定を防ぐ */
    float    still_accel_tol_g;  /**< 静止とみなす |加速度| の1gからのずれ [g] */
    uint32_t still_hold_ms;      /**< 静止がこの時間続いたら「静止と確定」 */
    float    still_window_s;     /**< 平均・ばらつきを見る窓の時定数 [s] */
    float    bias_tau_s;         /**< バイアス平均の時定数 [s]（確定中だけ更新） */
} attitude_params_t;

/**
 * 機体の配置（ミキサー用）と推力特性。単位はすべて g（1モーターあたりの推力）。
 * 各モーターの推力 = 全体推力 + sign[roll]*ロール指令 + sign[pitch]*ピッチ指令 + sign[yaw]*ヨー指令。
 */
typedef struct {
    /**
     * ★★★ TODO: プロペラを付けて回す前に必ず確定 ★★★
     * 行 = M1..M4、列 = roll,pitch,yaw の符号(+1/-1)。モーターの位置・回転方向が未決なので
     * 今は仮の値（M1=前左 M2=前右 M3=後右 M4=後左、M1とM3がCW）。実機で1つずつ確かめること。
     */
    int8_t  mixer_sign[FC_NUM_MOTORS][3];
    float   thrust_k_g_per_vduty;   /**< 推力[g] ≒ K × デューティ × 電圧[V]。HW実測 9.4（3.5〜4.2Vで直線） */
    float   voltage_min_calc_v;     /**< 換算に使う電圧の下限 [V]。ADC未接続・異常値で割り算が暴れないように（提案） */
    float   throttle_full_thrust_g; /**< スロットル1.0のときの1モーター推力 [g]（スロットル→推力の換算。提案） */
} airframe_params_t;

/** 電池。電圧範囲を含む。 */
typedef struct {
    float adc_divider_ratio; /**< 分圧比（電池電圧 = ADC電圧 × この値）。仮置き。TODO: HW設計書で確認 */
    float lpf_tau_s;         /**< 電圧のローパス時定数 [s]（ゆっくり。モーター負荷で電圧が揺れるため） */
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
    float    idle_throttle;        /**< 提案: 飛行中でもスロットルがこれ以下ならモーターを止める（0..1） */
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
    attitude_params_t    attitude;
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
