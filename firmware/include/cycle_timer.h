/**
 * @file cycle_timer.h
 * @brief 時計・1周期ごとの時刻計測・ウォッチドッグの薄いラッパ。SDK非依存。
 *
 * ポート（差し替え境界）: fc_clock_* と fc_wdt_* は実機は port_pico ディレクトリの clock_wdt_pico.c、
 * シミュレータは同じ関数を自前の.cで実装する（時計は仮想時間でよい）。
 * cycle_timing_begin/end は純粋C（src/cycle_timer.c）で、fc_clock_now_us32() だけを使う。
 *
 * Pico SDK の watchdog_* 等と名前がぶつからないよう fc_ を付ける。
 * 実機の実装はSDK（time_us_64, watchdog_*）を呼ぶ。
 */
#ifndef CYCLE_TIMER_H
#define CYCLE_TIMER_H

#include <stdint.h>
#include <stdbool.h>

/** INT1が来ないと判断する時間 [µs]（設計メモ2章: 約5ms）。 */
#define FC_INT1_TIMEOUT_US 5000u

/** 1周期ぶんの時刻記録（ログに入る）。 */
typedef struct {
    uint32_t cycle_index;      /**< 周期番号（cycle_timing_begin のたびに+1。ゼロ初期化から始めると最初の周期は1） */
    uint32_t start_us;         /**< 周期の開始時刻 = INT1の立ち上がり時刻（下位32bit） */
    uint16_t wake_latency_us;  /**< INT1から処理開始までの遅れ */
    uint16_t proc_us;          /**< 処理開始〜PWM更新完了（1.04msに収まるかの確認用） */
    uint16_t int1_missed;      /**< この周期までに取りこぼしたINT1の数（回数差-1） */
    uint32_t t_begin_us;       /**< 処理開始時刻（内部用） */
} cycle_timing_t;

/** 起動からの経過時間 [µs]（64bit）。コア0/1どちらでも可。随時。 */
uint64_t fc_clock_now_us64(void);

/** 同、下位32bit。約71分で一周する。差分計算専用。 */
uint32_t fc_clock_now_us32(void);

/**
 * 周期の開始を記録する。
 * 呼ぶコア: コア0。頻度: 1周期に1回、INT1待ちから戻った直後。
 * @param int1_timestamp_us  imu_int1_event_t.timestamp_us
 * @param int1_count_delta   前回からのINT1回数の増分（2以上なら取りこぼし）
 */
void cycle_timing_begin(cycle_timing_t *t, uint32_t int1_timestamp_us, uint32_t int1_count_delta);

/** 周期の終了（PWM更新直後）を記録し proc_us を確定する。コア0、1周期に1回。 */
void cycle_timing_end(cycle_timing_t *t);

/** ウォッチドッグを開始。コア0、起動処理が終わった時点で1回。timeout_msは params で決める。 */
void fc_wdt_start(uint32_t timeout_ms);

/** ウォッチドッグを餌やり。コア0だけが呼ぶ。1周期に1回、周期の最後。 */
void fc_wdt_feed(void);

/**
 * 直前のリセットがウォッチドッグ起因か（watchdog_enable_caused_reboot）。
 * コア0、起動時に1回。true なら非常停止（理由: WDT）へ（設計メモ5章）。
 */
bool fc_wdt_caused_reboot(void);

#endif /* CYCLE_TIMER_H */
