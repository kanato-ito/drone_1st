/**
 * @file bench_ipc_stub.c
 * @brief 【仮の代用品】mailbox.h / ipc_flags.h / ipc.h の ipc_flags() の最小実装。loop_bench 専用。
 *
 * ★ステップ④で置き換える★ 本物（オーナー作のシーケンスロック版メールボックスと、_Atomic の旗）が
 * src/ に入ったら、このファイルは捨てる。src/ をきれいに保つため、代用品はアプリのフォルダに置いた。
 *
 * 単一コア用: loop_bench はコア0だけで「PC役」と制御ループの両方を動かす。そのため排他はいらない。
 * 書き込みの seq は本物と同じ約束（偶数=安定、書くたびに +2、0=未受信）にした。
 * 旗は ipc_flags.h の約束（estop=立てるだけ、disarm_req=+1、disarm_ack=処理時に写す）どおり。
 */
#include <string.h>
#include "ipc.h"

static control_packet_t s_pkt;
static volatile uint32_t s_seq;
static ipc_flags_t s_flags;

int mailbox_write(const control_packet_t *src)
{
    s_pkt = *src;
    s_seq += 2u;
    return 0;
}

void mailbox_read(control_packet_t *dst) { *dst = s_pkt; }
uint32_t mailbox_seq(void) { return s_seq; }

ipc_flags_t *ipc_flags(void) { return &s_flags; }

void ipc_flags_init(ipc_flags_t *f)
{
    atomic_store(&f->estop, 0u);
    atomic_store(&f->disarm_req, 0u);
    atomic_store(&f->disarm_ack, 0u);
}
void ipc_flags_request_estop(ipc_flags_t *f) { atomic_store(&f->estop, 1u); }
void ipc_flags_request_disarm(ipc_flags_t *f) { atomic_store(&f->disarm_req, atomic_load(&f->disarm_req) + 1u); }
bool ipc_flags_estop_requested(const ipc_flags_t *f) { return atomic_load(&f->estop) != 0u; }
bool ipc_flags_disarm_pending(const ipc_flags_t *f) { return atomic_load(&f->disarm_req) != atomic_load(&f->disarm_ack); }
void ipc_flags_disarm_acknowledge(ipc_flags_t *f) { atomic_store(&f->disarm_ack, atomic_load(&f->disarm_req)); }
