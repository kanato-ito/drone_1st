/**
 * @file flight_state.h
 * @brief 機体の状態6つと遷移、非常停止の理由（設計メモ5章）。
 *
 * コア0、制御ループの中で1周期に1回。遷移は「入力の構造体 → 次の状態」の
 * 純粋な形にして、PCで試験できるようにする。モーターを回してよいかの判断も
 * ここの結果（flight_fsm_motors_allowed）で一元化する。SDK非依存。
 */
#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FS_BOOT = 0,           /**< 起動・初期化（モーター停止） */
    FS_WAIT_COMM,          /**< 通信待機（停止） */
    FS_STANDBY,            /**< 飛行指示待機＝ディスアーム（停止。制御器はここで選ぶ） */
    FS_FLIGHT,             /**< 飛行（人の指示で回る） */
    FS_EMERGENCY_DESCENT,  /**< 非常時降下（機体が回す、必ずPID） */
    FS_EMERGENCY_STOP      /**< 非常停止（停止。電源入れ直しのみ復帰） */
} flight_state_t;

/** 非常停止の理由。メモの例（IMU故障、通信途絶、WDT）に加え、提案を含む。 */
typedef enum {
    ESTOP_NONE = 0,
    ESTOP_IMU_INIT_FAIL,     /**< 起動時にIMU無応答（メモ5章） */
    ESTOP_WDT_REBOOT,        /**< ウォッチドッグ再起動（メモ5章） */
    ESTOP_IMU_TIMEOUT,       /**< 飛行中にINT1が約5ms来ない（IMU途絶） */
    ESTOP_TILT_EXCEEDED,     /**< 傾きが大きすぎる（仮60°） */
    ESTOP_OPERATOR,          /**< 操縦パケットの非常停止旗（提案） */
    ESTOP_DESCENT_LANDED,    /**< 非常時降下で着地を検出（提案） */
    ESTOP_DESCENT_TIMEOUT,   /**< 非常時降下が上限時間に達した（提案） */
    ESTOP_INTERNAL_ERROR     /**< 提案: 状態値が不正（メモリ破損など）。安全側に止める */
} estop_reason_t;

/** 非常時降下に入った原因。最終的に非常停止になってもログに両方残す（提案、#5）。 */
typedef enum {
    DESCENT_NONE = 0,
    DESCENT_COMM_LOST,
    DESCENT_BATTERY_LOW
} descent_reason_t;

/** 遷移判定の入力。制御ループが1周期の頭で集めて渡す。 */
typedef struct {
    bool  boot_imu_ok;          /**< BOOT中のみ有効: imu_init 成功 */
    bool  wdt_reboot;           /**< BOOT中のみ有効 */
    bool  imu_timeout;          /**< INT1が来ない、または読み出し連続失敗 */
    bool  comm_connected;       /**< 提案: 最初の有効パケットを受けた */
    bool  comm_lost;            /**< 新しいseqが comm_timeout_ms 来ない */
    bool  arm_rising_edge;      /**< アーム要求旗の0→1 */
    bool  level_and_zero_throttle; /**< 水平・スロットル0 */
    bool  disarm_request;
    bool  estop_request;        /**< 旗(ipc_flags)または操縦パケットの旗 */
    bool  battery_low;
    float tilt_rad;
    bool  landed;               /**< 非常時降下の着地検出 */
    bool  descent_timeout;      /**< 非常時降下の上限時間 */
} flight_fsm_inputs_t;

typedef struct {
    flight_state_t   state;
    estop_reason_t   estop_reason;
    descent_reason_t descent_reason;
    uint64_t         state_enter_us;
} flight_fsm_t;

/** 初期化（FS_BOOT）。コア0、起動時に1回。 */
void flight_fsm_init(flight_fsm_t *f, uint64_t now_us);

/**
 * 入力から次の状態を決める。
 * 呼ぶコア: コア0。頻度: 1周期に1回。状態が変わったら true。
 * 非常停止からは抜けない。tilt_estop_rad は引数で受ける（params依存を避ける）。
 * 遷移表は flight_state.c の先頭と docs/step3b_state_safety.md。評価順は上から（非常停止系が先）。
 * 傾きが NaN のときは「傾き過大」として扱う（安全側）。
 */
bool flight_fsm_step(flight_fsm_t *f, const flight_fsm_inputs_t *in, float tilt_estop_rad, uint64_t now_us);

/** この状態でモーターを回してよいか（FLIGHT と EMERGENCY_DESCENT のみ true）。コア0、毎周期。 */
bool flight_fsm_motors_allowed(flight_state_t s);

/** 状態名（ログ・USB表示用）。コア0/1どちらでも可。 */
const char *flight_state_name(flight_state_t s);
const char *estop_reason_name(estop_reason_t r);
const char *descent_reason_name(descent_reason_t r);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_STATE_H */
