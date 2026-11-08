/**
 * @file test_ipc.c
 * @brief mailbox（シーケンスロック）と ipc_flags の本物の試験。コア1の役は試験が演じる。
 */
#include "mailbox.h"
#include "ipc_flags.h"
#include "ipc.h"
#include "test_common.h"

static control_packet_t mk(float thr, uint32_t flags, uint32_t n)
{
    control_packet_t p = { 0.1f, -0.2f, 0.3f, thr, flags, n };
    return p;
}

static void test_mailbox_roundtrip(void)
{
    /* 未書き込み: seq=0、読むと全部0 */
    CHECK(mailbox_seq() == 0 || mailbox_seq() % 2 == 0);
    mailbox_test_force_seq(0);
    control_packet_t z; memset(&z, 0, sizeof(z)); mailbox_write(&z); mailbox_test_force_seq(0);
    control_packet_t r = mk(9, 9, 9);
    CHECK(mailbox_read(&r));
    CHECK(r.throttle == 0.0f && r.flags == 0 && r.packet_seq == 0);
    CHECK(mailbox_seq() == 0);

    /* 書いて読む。seq は書くたびに +2、常に偶数 */
    control_packet_t a = mk(0.5f, CTRL_FLAG_ARM_REQ | CTRL_FLAG_TAKEOVER_PID, 7);
    CHECK(mailbox_write(&a) == 0);
    CHECK(mailbox_seq() == 2);
    CHECK(mailbox_read(&r));
    CHECK(memcmp(&a, &r, sizeof(a)) == 0);
    control_packet_t b = mk(0.25f, CTRL_FLAG_DISARM, 8);
    mailbox_write(&b);
    CHECK(mailbox_seq() == 4);
    CHECK(mailbox_read(&r));
    CHECK(memcmp(&b, &r, sizeof(b)) == 0);              /* 上書き（最新だけ） */
    for (uint32_t i = 0; i < 100; i++) { control_packet_t c = mk(0.0f, 0, i); mailbox_write(&c); }
    CHECK(mailbox_seq() == 4 + 200);
    CHECK((mailbox_seq() & 1u) == 0);
    mailbox_read(&r);
    CHECK(r.packet_seq == 99);
}

static void test_mailbox_torn_read(void)
{
    control_packet_t a = mk(0.5f, 0, 1);
    mailbox_test_force_seq(0);
    mailbox_write(&a);
    uint32_t good = mailbox_seq();

    /* 書き込み途中（seqが奇数）: 読み直しを使い切って失敗し、dst は変えない */
    mailbox_test_force_seq(good + 1u);
    control_packet_t r = mk(7.0f, 7, 7);
    CHECK(!mailbox_read(&r));
    CHECK(r.throttle == 7.0f && r.flags == 7 && r.packet_seq == 7);
    CHECK(mailbox_seq() == good);                       /* 公開済みの seq は1つ前の偶数のまま（新着にならない） */

    /* 書き込みが終われば読める */
    mailbox_test_force_seq(good + 2u);
    CHECK(mailbox_read(&r));
    CHECK(mailbox_seq() == good + 2u);

    /* 本物の書き込みは奇数を残さない（write後は常に偶数） */
    for (int i = 0; i < 10; i++) { mailbox_write(&a); CHECK((mailbox_seq() & 1u) == 0); }
    CHECK(MAILBOX_READ_MAX_RETRIES >= 2);
}

static void test_flags_semantics(void)
{
    ipc_flags_t *f = ipc_flags();
    ipc_flags_init(f);
    CHECK(f != NULL && f == ipc_flags());
    CHECK(!ipc_flags_estop_requested(f));
    CHECK(!ipc_flags_disarm_pending(f));

    /* ディスアーム: カウンタ。何回来ても1回分の未処理、acknowledge で消える */
    ipc_flags_request_disarm(f);
    CHECK(ipc_flags_disarm_pending(f));
    ipc_flags_request_disarm(f); ipc_flags_request_disarm(f);
    CHECK(ipc_flags_disarm_pending(f));
    CHECK(atomic_load(&f->disarm_req) == 3 && atomic_load(&f->disarm_ack) == 0);
    ipc_flags_disarm_acknowledge(f);
    CHECK(!ipc_flags_disarm_pending(f));
    CHECK(atomic_load(&f->disarm_ack) == 3);
    ipc_flags_disarm_acknowledge(f);                    /* 冪等 */
    CHECK(!ipc_flags_disarm_pending(f));
    ipc_flags_request_disarm(f);                        /* 処理後に来た要求は取りこぼさない */
    CHECK(ipc_flags_disarm_pending(f));
    ipc_flags_disarm_acknowledge(f);

    /* 非常停止: 立てるだけ。ディスアームとは独立。何度立てても同じ */
    CHECK(!ipc_flags_estop_requested(f));
    ipc_flags_request_estop(f);
    CHECK(ipc_flags_estop_requested(f));
    ipc_flags_request_estop(f);
    CHECK(ipc_flags_estop_requested(f));
    ipc_flags_request_disarm(f);
    ipc_flags_disarm_acknowledge(f);
    CHECK(ipc_flags_estop_requested(f));                /* acknowledge は estop を下ろさない */
    ipc_flags_init(f);
    CHECK(!ipc_flags_estop_requested(f) && !ipc_flags_disarm_pending(f));

    /* カウンタの一周（32bit）でも pending = req != ack */
    atomic_store(&f->disarm_req, 0xFFFFFFFFu); atomic_store(&f->disarm_ack, 0xFFFFFFFFu);
    CHECK(!ipc_flags_disarm_pending(f));
    ipc_flags_request_disarm(f);
    CHECK(atomic_load(&f->disarm_req) == 0 && ipc_flags_disarm_pending(f));
    ipc_flags_init(f);
}

const test_t g_tests_ipc[] = {
    { "mailbox: 書き込み・読み出し・seq（+2、偶数）・未書き込み", test_mailbox_roundtrip },
    { "mailbox: 書き込み途中(奇数)は読み直し→失敗、dstを変えない", test_mailbox_torn_read },
    { "ipc_flags: estop / disarm カウンタの意味", test_flags_semantics },
};
const int g_tests_ipc_n = (int)(sizeof(g_tests_ipc) / sizeof(g_tests_ipc[0]));
