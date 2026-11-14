/**
 * @file ipc_flags.c
 * @brief 旗の本物（ipc_flags.h）。純粋C11、SDK非依存。
 *
 * どの語も書く人は1人（estop・disarm_req=コア1、disarm_ack=コア0）。32ビット単位の load/store だけを使い、
 * fetch_or 等の読み-変更-書きは使わない。estop は release で立て、acquire で読む。
 * 実体はここに1つ置き、ipc_flags()（ipc.h）で渡す。
 * 提案: ipc.c（ステップ④のキュー）ができたとき、ipc_flags() はそのまま残して ipc.c には移さない
 * （旗とキューを独立に試験できるようにするため）。
 */
#include "ipc.h"

static ipc_flags_t s_flags;

ipc_flags_t *ipc_flags(void) { return &s_flags; }

void ipc_flags_init(ipc_flags_t *f)
{
    atomic_store(&f->estop, 0u);
    atomic_store(&f->disarm_req, 0u);
    atomic_store(&f->disarm_ack, 0u);
}

void ipc_flags_request_estop(ipc_flags_t *f)
{
    atomic_store_explicit(&f->estop, 1u, memory_order_release);
}

void ipc_flags_request_disarm(ipc_flags_t *f)
{
    /* 書く人は1人なので load して store するだけでよい */
    uint32_t v = atomic_load_explicit(&f->disarm_req, memory_order_relaxed);
    atomic_store_explicit(&f->disarm_req, v + 1u, memory_order_release);
}

bool ipc_flags_estop_requested(const ipc_flags_t *f)
{
    return atomic_load_explicit(&f->estop, memory_order_acquire) != 0u;
}

bool ipc_flags_disarm_pending(const ipc_flags_t *f)
{
    return atomic_load_explicit(&f->disarm_req, memory_order_acquire) !=
           atomic_load_explicit(&f->disarm_ack, memory_order_relaxed);
}

void ipc_flags_disarm_acknowledge(ipc_flags_t *f)
{
    atomic_store_explicit(&f->disarm_ack, atomic_load_explicit(&f->disarm_req, memory_order_acquire),
                          memory_order_release);
}
