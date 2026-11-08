/**
 * @file cycle_timer.c
 * @brief cycle_timer.h の実装（ステップ②では「時計」の部分だけ）。
 *
 * 実装済み: fc_clock_now_us64 / fc_clock_now_us32
 * スタブ（今回は何もしない）: fc_wdt_start / fc_wdt_feed / fc_wdt_caused_reboot
 *   → ウォッチドッグはステップ③（制御ループ）で本実装する。今のうちに有効にすると、
 *     ブレッドボード試験中の printf などでリセットがかかって原因が分かりにくくなるため。
 * 未実装（呼ぶとリンクエラーになる）: cycle_timing_begin / cycle_timing_end
 *   → 周期ごとの時刻ログはステップ③で作る。
 */
#include "pico/stdlib.h"
#include "cycle_timer.h"

uint64_t fc_clock_now_us64(void)
{
    return time_us_64();   /* RP2350 のタイマー（1MHz）。コア0/1どちらから読んでも同じ値 */
}

uint32_t fc_clock_now_us32(void)
{
    return time_us_32();   /* 約71分で一周する。差分は (uint32_t)(後 - 前) で取ること */
}

/* ---- WDT: スタブ（ステップ③で実装）---- */
void fc_wdt_start(uint32_t timeout_ms)
{
    (void)timeout_ms;
}

void fc_wdt_feed(void)
{
}

bool fc_wdt_caused_reboot(void)
{
    return false;   /* スタブ: 常に「ウォッチドッグ起因ではない」 */
}
