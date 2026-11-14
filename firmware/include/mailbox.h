/**
 * @file mailbox.h
 * @brief 長さ1の上書きメールボックス（シーケンスロック）。コア1(書く)→コア0(読む)。
 *
 * 差し替え境界（ポート）ではない: 実装は src/mailbox.c（純粋C11、<stdatomic.h>。ステップ③bで実装済み）。
 * 書く側の割り込み禁止は呼ぶ側の責任（src/ はSDKに触れないため）。実機でコア1の割り込みの中で
 * 書くなら、呼ぶ前に自分の割り込みを止める。
 * シミュレータは本物をそのままリンクし、コア1の役を自分で演じる（mailbox_write を呼ぶ）。
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
#include <stdbool.h>
#include "control_packet.h"

/**
 * メールボックスへ書く（上書き）。
 * 呼ぶコア: コア1だけ（書く側は1つ。設計メモ4章）。頻度: 操縦パケット受信ごと（約50Hz）。
 * 中でコア1の割り込みを止め、seq を奇数→データ→偶数にする。戻り値は owner版どおり 0。
 */
int mailbox_write(const control_packet_t *src);

/** 読み直しの上限回数（提案。書き込みは短いので通常は1〜2回で済む）。 */
#define MAILBOX_READ_MAX_RETRIES 16u

/**
 * メールボックスから読む。書き込み途中(seqが奇数/変化)なら読み直す。
 * 提案: owner版は読めるまで無限に待つが、MAILBOX_READ_MAX_RETRIES 回で諦めて false を返す
 * （dst は変えない）。呼ぶ側は前回の値のまま続け、次の周期でまた読む。
 * 呼ぶコア: コア0。頻度: 1周期に1回（約940Hz）。
 * まだ一度も書かれていないときは true で全フィールド0が返る（seq が 0 で判別する）。
 */
bool mailbox_read(control_packet_t *dst);

/**
 * 公開済みの seq を返す（読むだけ。常に偶数、2ずつ増える＝書き込み回数×2。書き込み中は1つ前の値）。
 * 呼ぶコア: コア0。頻度: 1周期に1回。
 * 前回値と違えば「新しいパケットが来た」。通信途絶の判定(コア0)に使う。0 は未受信。
 */
uint32_t mailbox_seq(void);

/** 試験専用: seq を直接書き換える（書き込み途中=奇数の状態を作るため）。本番のコードからは呼ばない。 */
void mailbox_test_force_seq(uint32_t seq);

#endif /* MAILBOX_H */
