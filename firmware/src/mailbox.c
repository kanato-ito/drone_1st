/**
 * @file mailbox.c
 * @brief 長さ1の上書きメールボックス（シーケンスロック）の本物（mailbox.h）。純粋C11、SDK非依存。
 *
 * 書く人は1人（実機ではコア1）。seq が奇数の間は書き込み中、偶数なら安定。
 * 書く側: seq を奇数にする → fence(release) → データ → seq を偶数にする(release)。
 * 読む側: seq(acquire)が偶数か → データ → fence(acquire) → seq が同じか。違えば読み直し。
 * owner版の __dmb() は、ここでは atomic_thread_fence / memory_order の acquire-release に当たる。
 * データは32ビット語ごとの relaxed アトミックで読み書きする（書き込み中に読んでも未定義動作にならない）。
 *
 * 書く側の割り込み禁止（owner版 mailbox_write の「書く間は自分の割り込みを止める」）はここでは行わない
 * （SDKに触れないため）。実機でコア1の割り込みで書く場合は、呼ぶ側が割り込みを止めてから
 * mailbox_write を呼ぶこと。コア0が書く試験（loop_bench）では書く人が1人なので不要。
 */
#include <stdatomic.h>
#include <string.h>
#include "mailbox.h"

#define WORDS (sizeof(control_packet_t) / 4u)
_Static_assert(sizeof(control_packet_t) % 4u == 0u, "control_packet_t は4バイトの倍数");

static _Atomic uint32_t s_seq;
static _Atomic uint32_t s_data[WORDS];

int mailbox_write(const control_packet_t *src)
{
    uint32_t w[WORDS];
    memcpy(w, src, sizeof(w));

    uint32_t s = atomic_load_explicit(&s_seq, memory_order_relaxed);   /* 書く人は1人なので自分の値 */
    atomic_store_explicit(&s_seq, s + 1u, memory_order_relaxed);        /* 奇数: 書き込み中 */
    atomic_thread_fence(memory_order_release);
    for (unsigned i = 0; i < WORDS; i++) atomic_store_explicit(&s_data[i], w[i], memory_order_relaxed);
    atomic_store_explicit(&s_seq, s + 2u, memory_order_release);        /* 偶数: 安定 */
    return 0;
}

bool mailbox_read(control_packet_t *dst)
{
    for (unsigned tries = 0; tries < MAILBOX_READ_MAX_RETRIES; tries++) {
        uint32_t s1 = atomic_load_explicit(&s_seq, memory_order_acquire);
        if (s1 & 1u) continue;                                          /* 書き込み中 */
        uint32_t w[WORDS];
        for (unsigned i = 0; i < WORDS; i++) w[i] = atomic_load_explicit(&s_data[i], memory_order_relaxed);
        atomic_thread_fence(memory_order_acquire);
        uint32_t s2 = atomic_load_explicit(&s_seq, memory_order_relaxed);
        if (s1 == s2) {
            memcpy(dst, w, sizeof(w));                                  /* 未書き込み(seq=0)は全部0 */
            return true;
        }
    }
    return false;   /* 提案: 無限に待たず、失敗を返す。呼ぶ側は前回の値のまま次の周期で再試行 */
}

uint32_t mailbox_seq(void)
{
    /* 公開済みの seq。書き込み中(奇数)は1つ前の偶数を返す（新着の誤検出を防ぐ） */
    return atomic_load_explicit(&s_seq, memory_order_acquire) & ~1u;
}

void mailbox_test_force_seq(uint32_t seq)
{
    atomic_store_explicit(&s_seq, seq, memory_order_release);
}
