/**
 * @file log_record.h
 * @brief 飛行ログ1周期ぶんの固定長レコード（コア0が作り、キュー経由でコア1がPCへ送る）。
 *
 * 通し番号(seq)は捨てたときも進める。PC側は seq の飛びで欠落を検出する
 * （設計メモ4章）。dropped_total は「これまでに捨てた数」。
 * 1周期ごとの開始時刻と処理時間を含む（設計メモ2章）。SDK非依存。
 */
#ifndef LOG_RECORD_H
#define LOG_RECORD_H

#include <stdint.h>

#define LOG_FLAG_MOTORS_ON       (1u << 0)
#define LOG_FLAG_MIXER_SATURATED (1u << 1)
#define LOG_FLAG_FIFO_OVERRUN    (1u << 2)
#define LOG_FLAG_FIFO_TRUNCATED  (1u << 3)
#define LOG_FLAG_POLICY_ACTIVE   (1u << 4)  /**< 実際に方策の出力を使った */
#define LOG_FLAG_CMD_DROPPED     (1u << 5)  /**< 予備 */

typedef struct {
    uint32_t seq;             /**< 通し番号（push失敗でも進める） */
    uint32_t cycle_start_us;  /**< 周期開始 = INT1時刻（下位32bit。PC側でラップを展開） */
    uint32_t dropped_total;   /**< これまでに捨てたレコード数 */
    uint16_t wake_latency_us;
    uint16_t proc_us;         /**< 処理時間 */
    uint16_t policy_us;       /**< 方策の計算時間（見積もり0.18msの確認用） */
    uint16_t battery_mv;
    uint16_t int1_missed;
    uint8_t  fifo_words;      /**< 読んだFIFOワード数 */
    uint8_t  state;           /**< flight_state_t */
    uint8_t  estop_reason;    /**< estop_reason_t */
    uint8_t  descent_reason;  /**< descent_reason_t */
    uint8_t  active_controller; /**< controller_id_t（実際に使った方） */
    uint8_t  takeover_reason; /**< takeover_reason_t */
    uint8_t  flags;           /**< LOG_FLAG_* */
    uint8_t  reserved;
    float attitude[3];        /**< roll, pitch, yaw [rad] */
    float rate[3];            /**< [rad/s] */
    float target_att[3];
    float target_thrust;
    float output[4];          /**< 推力, トルク3 */
    float duty[4];            /**< M1..M4 */
} log_record_t;

_Static_assert(sizeof(log_record_t) % 4 == 0, "log_record_t は4バイト境界");
_Static_assert(sizeof(log_record_t) <= 128, "log_record_t が大きすぎる");

#endif /* LOG_RECORD_H */
