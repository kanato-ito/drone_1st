/**
 * @file mailbox.h
 * @brief 長さ1の上書きメールボックス（シーケンスロック）。コア1(書く)→コア0(読む)。
 *
 * ★オーナー実装に合わせて調整予定★
 * このヘッダは、オーナー実装 mail_box.c（mailbox_write / mailbox_read、
 * グローバル seq と data、書く側は割り込み禁止、seqの前後に __dmb()）の
 * 形に合わせた最小のインターフェース。実装(.c)はオーナーのコードに差し替える。
 *
 * owner版からの変更点（すべて提案。architecture.md の「メールボックスの切り出し案」）:
 *  - 型名 Drone → control_packet_t（アーム旗などを追加するため）
 *  - グローバル seq/data は mailbox.c 内の static にし、extern しない
 *  - mailbox_write の引数を値渡し → const ポインタ渡し（サイズが大きくなるため）
 *  - mailbox_seq() を追加（読んだ値が新しいか、通信途絶の判定に必要）
 *  - mailbox_read は owner版どおり「読めるまで待つ」。書き込みは短く割り込み禁止なので有限
 */
#ifndef MAILBOX_H
#define MAILBOX_H

#include <stdint.h>
#include "control_packet.h"

/**
 * メールボックスへ書く（上書き）。
 * 呼ぶコア: コア1だけ（書く側は1つ。設計メモ4章）。頻度: 操縦パケット受信ごと（約50Hz）。
 * 中でコア1の割り込みを止め、seq を奇数→データ→偶数にする。戻り値は owner版どおり 0。
 */
int mailbox_write(const control_packet_t *src);

/**
 * メールボックスから読む。書き込み途中(seqが奇数/変化)なら読み直す。
 * 呼ぶコア: コア0。頻度: 1周期に1回（約940Hz）。
 * まだ一度も書かれていないときは全フィールド0が返る（seq が 0 で判別する）。
 */
void mailbox_read(control_packet_t *dst);

/**
 * 現在の seq を返す（読むだけ。偶数なら安定、2ずつ増える＝書き込み回数×2）。
 * 呼ぶコア: コア0。頻度: 1周期に1回。
 * 前回値と違えば「新しいパケットが来た」。通信途絶の判定(コア0)に使う。0 は未受信。
 */
uint32_t mailbox_seq(void);

#endif /* MAILBOX_H */
