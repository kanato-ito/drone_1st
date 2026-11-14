/**
 * @file clock_wdt_pico.c
 * @brief cycle_timer.h の時計とウォッチドッグ（ポート: Pico SDK 版）。
 *
 * 時計: fc_clock_now_us64 / fc_clock_now_us32
 * ウォッチドッグ（ステップ③bで本実装）: fc_wdt_start / fc_wdt_feed / fc_wdt_caused_reboot
 *   Pico SDK の watchdog_enable / watchdog_update / watchdog_enable_caused_reboot を呼ぶだけの薄い層。
 *   fc_wdt_start は制御ループの初期化が済んだ最後に1回だけ呼ぶ（control_loop_init）。
 *   餌やりはコア0の control_loop_run_once が毎周期行う（非常停止中も）。
 * 周期ごとの時刻計測（cycle_timing_*）は SDK に触らないので src/cycle_timer.c にある。
 */
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "cycle_timer.h"

uint64_t fc_clock_now_us64(void)
{
    return time_us_64();   /* RP2350 のタイマー（1MHz）。コア0/1どちらから読んでも同じ値 */
}

uint32_t fc_clock_now_us32(void)
{
    return time_us_32();   /* 約71分で一周する。差分は (uint32_t)(後 - 前) で取ること */
}

/* ---- WDT ---- */
void fc_wdt_start(uint32_t timeout_ms)
{
    if (timeout_ms == 0) timeout_ms = 1;
    /* 第2引数 true: デバッガで止めている間はカウントを止める */
    watchdog_enable(timeout_ms, true);
}

void fc_wdt_feed(void)
{
    watchdog_update();
}

bool fc_wdt_caused_reboot(void)
{
    return watchdog_enable_caused_reboot();   /* watchdog_enable のタイムアウトによる再起動か */
}
