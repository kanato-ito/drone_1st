/**
 * @file cycle_timer.c
 * @brief 1周期ごとの時刻計測（cycle_timer.h）。SDK非依存。
 *
 * 時計（fc_clock_now_*）とウォッチドッグは差し替え境界（ポート）側
 * （実機: port_pico ディレクトリの clock_wdt_pico.c）。ここは fc_clock_now_us32() だけを使う。
 */
#include "cycle_timer.h"

static uint16_t sat_u16(uint32_t v)
{
    return (uint16_t)(v > 65535u ? 65535u : v);
}

void cycle_timing_begin(cycle_timing_t *t, uint32_t int1_timestamp_us, uint32_t int1_count_delta)
{
    t->start_us = int1_timestamp_us;
    t->t_begin_us = fc_clock_now_us32();
    t->wake_latency_us = sat_u16((uint32_t)(t->t_begin_us - int1_timestamp_us));
    t->int1_missed = sat_u16(int1_count_delta > 1u ? int1_count_delta - 1u : 0u);
    t->cycle_index++;   /* 初期化は呼び出し側で0にしておく。最初の周期の番号は1 */
}

void cycle_timing_end(cycle_timing_t *t)
{
    t->proc_us = sat_u16((uint32_t)(fc_clock_now_us32() - t->t_begin_us));
}
