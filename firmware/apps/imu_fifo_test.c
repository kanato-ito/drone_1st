/**
 * @file imu_fifo_test.c
 * @brief ステップ②のブレッドボード試験: IMU の FIFO を INT1 で起こして読む。
 *
 * 起動時に SPI の実クロック・WHO_AM_I・設定の読み戻しを表示し、そのあと
 * INT1 を待つ → FIFO を全部読む、を繰り返す。画面には1秒に1行だけ出す
 * （printf は遅いので、ループの時間を乱さないため）。
 *
 * ---- 合格条件（ボードを静止させて数十秒見る。数値の許容幅は提案）----
 *  1. INT1 周波数       : 約940Hz（920〜980Hz）。約1880Hz なら WTM の単位の読み違い
 *                         （ワードではなく「組」と読んでいる）。
 *  2. 1回に読むワード数 : 平均 約16（16〜18）、最小 16。多すぎる/増え続けるなら読み遅れ。
 *  3. 組数              : 1秒あたり 約7500（ODR 7.68kHz。実測ODRは約7519Hz）
 *  4. OVR（FIFOあふれ） : 0 のまま
 *  5. タイムアウト      : 0 のまま（INT1が5ms来なかった回数）
 *  6. ERR（読み出し失敗）: 0 のまま
 *  7. 静止時の加速度ノルム |a| : 約1.00g（0.95〜1.05g）
 *  8. imu_read_fifo の所要時間 : 平均・最大を記録しておく（目安は約160µs、設計メモ2章の
 *                         1.04msに収まるかの判断材料。合否ではなく記録）
 *  9. MISS（読み終わった後でINT1がHighのまま）: 記録（0に近いほど良い）
 *
 * 表示の見方は docs/step2_imu_fifo.md を参照。
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"

#include "board_pins.h"
#include "fc_types.h"
#include "cycle_timer.h"
#include "imu_lsm6dsv16x.h"

#define REPORT_PERIOD_US 1000000u
#define RAD_TO_DEG       57.29577951308232f

/* 1秒ぶんの統計（表示のたびにクリアする） */
typedef struct {
    uint32_t reads;
    uint64_t words_sum;
    uint32_t words_min;
    uint32_t words_max;
    uint64_t pairs_sum;
    uint64_t read_us_sum;
    uint32_t read_us_max;
} window_stats_t;

static void window_clear(window_stats_t *w)
{
    w->reads = 0;
    w->words_sum = 0;
    w->words_min = 0xFFFFFFFFu;
    w->words_max = 0;
    w->pairs_sum = 0;
    w->read_us_sum = 0;
    w->read_us_max = 0;
}

static void print_startup(imu_status_t st)
{
    printf("\n=== imu_fifo_test (step 2: LSM6DSV16X FIFO) ===\n");
    printf("SPI: 指定 %u Hz -> 実際 %u Hz\n",
           (unsigned)BOARD_IMU_SPI_BAUD_REQUEST_HZ, (unsigned)imu_spi_actual_baud_hz());
    printf("imu_init: %s (%d)\n",
           st == IMU_OK ? "OK" :
           st == IMU_ERR_WHOAMI ? "WHO_AM_I不一致（配線/電源/CSを確認）" :
           st == IMU_ERR_SPI ? "SPI失敗" : "設定の読み戻しが不一致", (int)st);

    imu_reg_readback_t rb[IMU_READBACK_MAX];
    uint8_t n = imu_config_readback(rb, IMU_READBACK_MAX);
    printf("設定の読み戻し (読んだ値 / 期待値):\n");
    for (uint8_t i = 0; i < n; i++) {
        bool ok = ((rb[i].value ^ rb[i].expected) & rb[i].mask) == 0;
        printf("  0x%02X %-17s = 0x%02X / 0x%02X  %s\n",
               rb[i].addr, rb[i].name, rb[i].value, rb[i].expected, ok ? "OK" : "NG");
    }
}

int main(void)
{
    stdio_init_all();

    /* USB のシリアルモニタが開かれるまで待つ（開く前の printf は捨てられるため） */
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);

    imu_config_t cfg = imu_config_default();
    imu_status_t st = imu_init(&cfg);
    print_startup(st);
    if (st != IMU_OK) {
        /* 失敗したら止まって、何度でも原因を表示する（配線を直してリセットボタンで再試験） */
        for (;;) {
            sleep_ms(2000);
            printf("imu_init 失敗 (%d)。配線を直して電源を入れ直してください\n", (int)st);
        }
    }

    /* バッチは大きい（32組×24バイト）ので static に置く（スタックを使わない） */
    static imu_batch_t batch;

    /* 累計（起動からの合計。0 のままが合格） */
    uint32_t total_ovr = 0, total_timeouts = 0, total_errors = 0, total_miss = 0;

    window_stats_t w;
    window_clear(&w);

    imu_sample_t latest = {{0, 0, 0}, {0, 0, 1}};
    imu_int1_event_t ev = {0, 0};
    uint32_t last_report_us = fc_clock_now_us32();
    uint32_t last_report_count = ev.count;
    bool count_valid = false;   /* 最初の INT1 を受け取るまで周波数の基準が無い */
    uint32_t sec = 0;

    for (;;) {
        bool got = imu_wait_int1(FC_INT1_TIMEOUT_US, &ev);
        bool do_read = got;
        if (got) {
            if (!count_valid) {
                last_report_count = ev.count - 1u;   /* 最初の1回を数に入れる */
                count_valid = true;
            }
        } else {
            total_timeouts++;
            /* ピンがHighのままだと次の立ち上がりが来ない。その場合は読んで救出する */
            do_read = imu_int1_pin_is_high();
        }

        if (do_read) {
            uint32_t t0 = fc_clock_now_us32();
            imu_status_t rs = imu_read_fifo(&batch);
            uint32_t dt = (uint32_t)(fc_clock_now_us32() - t0);

            if (rs != IMU_OK) {
                total_errors++;
            }
            if (batch.fifo_overrun) {
                total_ovr++;
            }
            /* 読み終わった後もINT1がHighなら、次の立ち上がりを取りこぼす */
            if (imu_int1_pin_is_high()) {
                total_miss++;
            }

            w.reads++;
            w.words_sum += batch.words_read;
            if (batch.words_read < w.words_min) w.words_min = batch.words_read;
            if (batch.words_read > w.words_max) w.words_max = batch.words_read;
            w.pairs_sum += batch.count;
            w.read_us_sum += dt;
            if (dt > w.read_us_max) w.read_us_max = dt;

            if (batch.count > 0) {
                latest = batch.samples[batch.count - 1];
            }
        }

        uint32_t now = fc_clock_now_us32();
        uint32_t elapsed = (uint32_t)(now - last_report_us);
        if (elapsed >= REPORT_PERIOD_US) {
            sec++;
            float int1_hz = count_valid
                ? (float)(uint32_t)(ev.count - last_report_count) * 1.0e6f / (float)elapsed
                : 0.0f;
            float words_avg = w.reads ? (float)w.words_sum / (float)w.reads : 0.0f;
            uint32_t words_min = w.reads ? w.words_min : 0;
            uint32_t read_avg = w.reads ? (uint32_t)(w.read_us_sum / w.reads) : 0;
            float gx = latest.gyro_rps.x * RAD_TO_DEG;
            float gy = latest.gyro_rps.y * RAD_TO_DEG;
            float gz = latest.gyro_rps.z * RAD_TO_DEG;
            float anorm = sqrtf(latest.accel_g.x * latest.accel_g.x +
                                latest.accel_g.y * latest.accel_g.y +
                                latest.accel_g.z * latest.accel_g.z);

            /* 1秒に1行だけ。OVR/TMO/ERR/MISS は起動からの累計 */
            printf("[%4us] INT1=%6.1fHz words avg/min/max=%.1f/%u/%u pairs=%u "
                   "OVR=%u TMO=%u ERR=%u MISS=%u read_us avg/max=%u/%u "
                   "gyro[dps]=(%+7.2f %+7.2f %+7.2f) accel[g]=(%+6.3f %+6.3f %+6.3f) |a|=%.3f\n",
                   (unsigned)sec, (double)int1_hz,
                   (double)words_avg, (unsigned)words_min, (unsigned)w.words_max,
                   (unsigned)w.pairs_sum,
                   (unsigned)total_ovr, (unsigned)total_timeouts, (unsigned)total_errors, (unsigned)total_miss,
                   (unsigned)read_avg, (unsigned)w.read_us_max,
                   (double)gx, (double)gy, (double)gz,
                   (double)latest.accel_g.x, (double)latest.accel_g.y, (double)latest.accel_g.z,
                   (double)anorm);

            window_clear(&w);
            /* 次の1秒の基準。printf にかかった時間は次の窓に含まれる（ずれは1行ごとに補正される） */
            last_report_us = now;
            last_report_count = ev.count;
        }
    }
}
