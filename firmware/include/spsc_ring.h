/**
 * @file spsc_ring.h
 * @brief 書く側1つ・読む側1つのロックなしリングバッファ（設計メモ4章）。
 *
 * 差し替え境界（ポート）ではない: 実装（ステップ④）は src/ の純粋C11（<stdatomic.h>）で書く。
 * シミュレータは本物をそのままリンクし、コア1の役を自分で演じる。
 *
 * - headは書く側だけ、tailは読む側だけが進める。要素は固定長、領域は呼び出し側が
 *   静的に用意して渡す（mallocなし）。容量は2のべき乗。
 * - 満杯のときの扱いはこの層では決めない。pushは false を返すだけ。
 *     ログ(コア0→1): 新しいほうを捨てて捨てた数を数える（ipc.h の ipc_log_push）
 *     コマンド(コア1→0): コア1が空くまで待つ（ipc.h の ipc_cmd_push_wait）
 *   古いほうを捨てない理由: 書く側がtailに触ると前提が崩れる（メモ4章）。
 */
#ifndef SPSC_RING_H
#define SPSC_RING_H

/* C専用ヘッダ（<stdatomic.h> / _Atomic を使う）。thrust_model / mixer / controller_pid / attitude の
 * C++版はこのヘッダに依存しない（推移的 include も無い）。C++から include しない。 */
#ifdef __cplusplus
#error "C専用ヘッダ: C++から include しない"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *buf;            /**< 呼び出し側が確保した capacity*elem_size バイト */
    uint32_t elem_size;
    uint32_t capacity;       /**< 2のべき乗 */
    _Atomic uint32_t head;   /**< 書く側だけが更新（通し番号。mask で添字化） */
    _Atomic uint32_t tail;   /**< 読む側だけが更新 */
} spsc_ring_t;

/** 初期化。呼ぶコア: コア0、コア1起動前に1回。capacity が2のべき乗でなければ false。 */
bool spsc_ring_init(spsc_ring_t *r, void *storage, uint32_t elem_size, uint32_t capacity);

/** 1要素を書く。満杯なら false（何も書かない）。書く側のコアだけ。頻度: 用途による。 */
bool spsc_ring_push(spsc_ring_t *r, const void *elem);

/** 1要素を読む。空なら false。読む側のコアだけ。 */
bool spsc_ring_pop(spsc_ring_t *r, void *elem);

/** 現在の要素数の目安（どちらのコアからでも可。参考値）。 */
uint32_t spsc_ring_count(const spsc_ring_t *r);

#ifdef __cplusplus
}
#endif

#endif /* SPSC_RING_H */
