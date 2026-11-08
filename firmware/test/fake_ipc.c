/**
 * @file fake_ipc.c
 * @brief mailbox / ipc_flags の偽物（ホスト試験用）。ステップ④で本物（src/）ができるまでの代わり。
 *
 * 試験が「コア1の役」を演じる: mailbox_write でパケットを書き、ipc_flags_request_* で旗を立てる。
 * 本物のヘッダ(mailbox.h, ipc_flags.h, ipc.h の ipc_flags())と同じ形で実装する。
 */
#include "ipc.h"
#include "fake_ipc.h"

static control_packet_t s_pkt;
static uint32_t s_seq;
static ipc_flags_t s_flags;

int mailbox_write(const control_packet_t *src) { s_pkt = *src; s_seq += 2u; return 0; }
void mailbox_read(control_packet_t *dst) { *dst = s_pkt; }
uint32_t mailbox_seq(void) { return s_seq; }

ipc_flags_t *ipc_flags(void) { return &s_flags; }

void ipc_flags_init(ipc_flags_t *f)
{
    atomic_store(&f->estop, 0u); atomic_store(&f->disarm_req, 0u); atomic_store(&f->disarm_ack, 0u);
}
void ipc_flags_request_estop(ipc_flags_t *f) { atomic_store(&f->estop, 1u); }
void ipc_flags_request_disarm(ipc_flags_t *f) { atomic_store(&f->disarm_req, atomic_load(&f->disarm_req) + 1u); }
bool ipc_flags_estop_requested(const ipc_flags_t *f) { return atomic_load(&f->estop) != 0u; }
bool ipc_flags_disarm_pending(const ipc_flags_t *f) { return atomic_load(&f->disarm_req) != atomic_load(&f->disarm_ack); }
void ipc_flags_disarm_acknowledge(ipc_flags_t *f) { atomic_store(&f->disarm_ack, atomic_load(&f->disarm_req)); }

void fake_ipc_reset(void)
{
    memset(&s_pkt, 0, sizeof(s_pkt));
    s_seq = 0;
    ipc_flags_init(&s_flags);
}
