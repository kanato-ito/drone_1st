/**
 * @file comm_core1.h
 * @brief コア1: Wi-Fi（UDP操縦・ログ、TCP設定）とUSB。
 *
 * Wi-FiとUSBの初期化は必ずコア1で行う（その割り込みをコア1に届けるため、設計メモ3章）。
 * コア0から multicore_launch_core1(comm_core1_entry) で起動する。
 * Wi-FiがAPかステーションか、ポート番号、USBの用途はメモに無く未決（architecture.md）。
 * SDK非依存に保つため、cyw43/lwIP/tinyusb の型はこのヘッダに出さない。
 */
#ifndef COMM_CORE1_H
#define COMM_CORE1_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "control_packet.h"
#include "log_record.h"
#include "cmd_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * コア1の入口。Wi-Fi・USBを初期化し、以後 UDP受信→mailbox_write、
 * ログ取り出し→UDP送信、TCP→コマンドキュー を繰り返す。戻らない。
 * 呼ぶコア: コア1（起動）。頻度: 起動時に1回。
 * 通信途絶の判定はここではしない（コア0の仕事。コア1が止まっても止められるように）。
 */
void comm_core1_entry(void);

/**
 * 受信バイト列を操縦パケットに変換。長さ・形式・(提案)CRCが不正なら false。
 * 純粋関数（ネットワーク非依存）なのでPCで試験できる。
 * 呼ぶコア: コア1。頻度: UDPパケット受信ごと（約50Hz）。
 */
bool comm_decode_control_packet(const uint8_t *buf, size_t len, control_packet_t *out);

/**
 * ログレコードを送信用バイト列に変換。書いたバイト数、入らなければ0。
 * 純粋関数。コア1、ログ送信ごと。形式（リトルエンディアンの生構造体+バージョン等）は提案、未決。
 */
size_t comm_encode_log_record(const log_record_t *rec, uint8_t *buf, size_t cap);

/**
 * TCPで受けた設定メッセージをコマンドに変換する（純粋関数）。不正なら false。
 * 呼ぶコア: コア1。頻度: 設定受信時のみ。
 */
bool comm_decode_tcp_command(const uint8_t *buf, size_t len, cmd_msg_t *out);

#ifdef __cplusplus
}
#endif

#endif /* COMM_CORE1_H */
