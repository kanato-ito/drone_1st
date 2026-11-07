/**
 * @file ipc.h
 * @brief コア間の受け渡しを1か所に集める。
 *
 * | データ                 | 向き  | 方式                     |
 * | 操縦(目標・アーム等)   | 1→0  | mailbox.h                |
 * | 非常停止・ディスアーム | 1→0  | ipc_flags.h              |
 * | コマンド               | 1→0  | spsc_ring（満杯: 待つ）  |
 * | 返信(受付/拒否, 提案)  | 0→1  | spsc_ring（満杯: 捨てる）|
 * | 飛行ログ               | 0→1  | spsc_ring（満杯: 新しいほうを捨て数える）|
 *
 * 実体は ipc.c の静的領域。コア1を起動する前にコア0で ipc_init() を呼ぶ。
 */
#ifndef IPC_H
#define IPC_H

#include <stdint.h>
#include <stdbool.h>
#include "mailbox.h"
#include "ipc_flags.h"
#include "spsc_ring.h"
#include "log_record.h"
#include "cmd_msg.h"

#define IPC_LOG_QUEUE_CAPACITY   128 /**< 提案: 約136ms分。2のべき乗 */
#define IPC_CMD_QUEUE_CAPACITY    32
#define IPC_REPLY_QUEUE_CAPACITY  16
/** 1周期に処理するコマンドの上限（提案。周期を伸ばさないため） */
#define IPC_CMD_MAX_PER_CYCLE      4

/** 旗とキューを初期化。コア0、コア1起動前に1回。 */
void ipc_init(void);

/** 旗の実体。コア1は ipc_flags_request_*、コア0は *_requested / *_pending / disarm_acknowledge。 */
ipc_flags_t *ipc_flags(void);

/**
 * ログを1件積む。seq と dropped_total を内部で付ける。満杯なら捨てて数える。
 * 呼ぶコア: コア0だけ。頻度: 1周期に1回（約940Hz）。待たない。
 * @return 積めたら true、捨てたら false
 */
bool ipc_log_push(log_record_t *rec);

/** ログを1件取り出す。コア1、送信ループで空になるまで。 */
bool ipc_log_pop(log_record_t *rec);

/**
 * コマンドを積む。満杯なら空くまで待つ（メモ4章）が、timeout_us で諦めて false を返す。
 * 待っている間もネットワークの処理が止まらないよう、呼び出し側が再試行する形を提案（#8）。
 * 呼ぶコア: コア1だけ。頻度: TCPで設定を受けたときのみ。
 */
bool ipc_cmd_push_wait(const cmd_msg_t *cmd, uint32_t timeout_us);

/** コマンドを1件取り出す。コア0、ループの最初に1周期あたり IPC_CMD_MAX_PER_CYCLE 件まで。 */
bool ipc_cmd_pop(cmd_msg_t *cmd);

/** 返信を積む（満杯なら捨てる）。コア0、コマンド処理のたびに。 */
bool ipc_reply_push(const cmd_reply_t *r);

/** 返信を取り出す。コア1、TCP処理の中で。 */
bool ipc_reply_pop(cmd_reply_t *r);

#endif /* IPC_H */
