/**
 * @file control_packet.h
 * @brief 操縦パケットの中身（コア1が受信し、メールボックスでコア0へ渡す）。SDK非依存。
 *
 * オーナー実装の Drone {roll, pitch, yaw, throttle} を拡張した提案型。
 * 差分: アーム要求・ディスアーム・非常停止の旗と、通し番号を追加
 * （設計メモ4章・6章。architecture.md の未決事項 #3 を参照）。
 * これは「デコード済みの内部表現」。ネットワーク上の形式(ワイヤ形式)は
 * comm_core1.h の encode/decode が担当し、メモには未記載（提案）。
 */
#ifndef CONTROL_PACKET_H
#define CONTROL_PACKET_H

#include <stdint.h>

/** control_packet_t.flags のビット。操縦パケットの旗として毎回送られる（設計メモ6章）。 */
#define CTRL_FLAG_ARM_REQ     (1u << 0) /**< アーム要求。ボタンを押している間ずっと1。0→1の変化でアーム判定 */
#define CTRL_FLAG_DISARM      (1u << 1) /**< ディスアーム */
#define CTRL_FLAG_ESTOP       (1u << 2) /**< 非常停止 */
#define CTRL_FLAG_TAKEOVER_PID (1u << 3) /**< 提案: 「人の指示」でPIDに取り上げさせる（メモ5章にあるがパケットに項目なし） */

typedef struct {
    float    roll;      /**< 目標ロール [rad]（owner版 Drone.roll と同じ名前） */
    float    pitch;     /**< 目標ピッチ [rad] */
    float    yaw;       /**< 目標ヨー [rad]（角度か角速度かは未決） */
    float    throttle;  /**< スロットル 0..1（owner版と同じ名前。N への換算は params） */
    uint32_t flags;     /**< CTRL_FLAG_* */
    uint32_t packet_seq;/**< 送信側の通し番号（欠落の統計用。メールボックスのseqとは別物） */
} control_packet_t;

#endif /* CONTROL_PACKET_H */
