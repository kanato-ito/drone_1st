/**
 * @file port_sim.c
 * @brief ポート（差し替え境界）の偽物。実機の port_pico ディレクトリの4ファイルと同じ関数を、SDKなしで実装する。
 *
 * 守ること（docs/hw_boundary.md 3章）:
 *  - IMU: imu_read_fifo は imu_batch_t を本物と同じ単位（rad/s、g、IMU基板の軸）で埋める。
 *         サンプルは Python が fcsim_imu_push で1つずつ積む（物理の1刻み = IMUの1サンプル）。
 *         imu_sample_period_s は本物と同じ（imu_config_default の実測ODR 7519Hz から）。
 *  - PWM: motor_pwm_set で受けたデューティ（0..1、範囲外とNaNは0）を覚える。stop で全部0。
 *  - 時計: 仮想時間。fcsim_set_time_us で進める。us32 は us64 の下位32bit。WDTは回数を数えるだけ。
 *  - 電池ADC: 12bitの生の値を返す。電圧への換算は battery.c の仕事なのでここではしない。
 */
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "imu_lsm6dsv16x.h"
#include "motor_pwm.h"
#include "cycle_timer.h"
#include "batt_adc.h"
#include "params.h"
#include "fcsim.h"

/* ---------------- IMU ---------------- */

/* FIFOの代わりのリングバッファ。本物のFIFOは約4.5KB（数百ワード）なので、それより十分大きく取る。 */
#define SIM_FIFO_CAP 1024

static imu_sample_t s_fifo[SIM_FIFO_CAP];
static int s_fifo_head;   /* 次に書く位置 */
static int s_fifo_count;
static bool s_fifo_overrun;
static imu_config_t s_imu_cfg;
static imu_int1_event_t s_int1;
static bool s_int1_pin_high;   /* 試験用: INT1ピンの張り付き（fcsim_set_int1_pin_high） */
static uint32_t s_read_fail;   /* 試験用: この回数だけ imu_read_fifo を失敗させる */

imu_config_t imu_config_default(void)
{
    /* port_pico/imu_lsm6dsv16x.c の imu_config_default と同じ値にする（ずれると dt が変わる） */
    imu_config_t c;
    c.spi_baud_request_hz  = 8000000u;
    c.fifo_watermark_words = 16;
    c.odr_hz_measured      = 7519.0f;
    c.gyro_fs_dps          = 2000;
    c.accel_fs_g           = 16;
    return c;
}

imu_status_t imu_init(const imu_config_t *cfg)
{
    s_imu_cfg = *cfg;
    s_fifo_head = 0;
    s_fifo_count = 0;
    s_fifo_overrun = false;
    s_int1.count = 0;
    s_int1.timestamp_us = 0;
    s_int1_pin_high = false;
    s_read_fail = 0;
    return IMU_OK;
}

imu_status_t imu_whoami_check(void) { return IMU_OK; }

uint8_t imu_config_readback(imu_reg_readback_t *out, uint8_t max_entries)
{
    (void)out; (void)max_entries;
    return 0;   /* レジスタは無いので表示する行もない */
}

uint32_t imu_spi_actual_baud_hz(void) { return 7500000u; }

imu_status_t imu_fifo_flush(void)
{
    s_fifo_head = 0;
    s_fifo_count = 0;
    return IMU_OK;
}

void imu_on_int1_edge(void)
{
    s_int1.count++;
    s_int1.timestamp_us = fc_clock_now_us32();
}

bool imu_wait_int1(uint32_t timeout_us, imu_int1_event_t *ev)
{
    /* シミュレータは待たない。ウォーターマーク（ワード数÷2 = 組数）以上たまっていれば「来た」とする。 */
    (void)timeout_us;
    int wtm_sets = s_imu_cfg.fifo_watermark_words / 2;
    if (s_fifo_count >= wtm_sets && wtm_sets > 0) {
        imu_on_int1_edge();
        *ev = s_int1;
        return true;
    }
    return false;
}

bool imu_int1_pin_is_high(void) { return s_int1_pin_high; }

void fcsim_set_int1_pin_high(int32_t high) { s_int1_pin_high = high != 0; }

void fcsim_imu_fail_reads(uint32_t n) { s_read_fail = n; }

void fcsim_int1_edge(void) { imu_on_int1_edge(); }

void fcsim_get_int1_event(imu_int1_event_t *ev) { *ev = s_int1; }

imu_status_t imu_read_fifo(imu_batch_t *out)
{
    if (s_read_fail > 0u) {
        s_read_fail--;
        out->count = 0;
        out->words_read = 0;
        return IMU_ERR_SPI;   /* SPIの失敗の代わり。FIFOの中身は残る */
    }
    int n = s_fifo_count < IMU_BATCH_MAX_SAMPLES ? s_fifo_count : IMU_BATCH_MAX_SAMPLES;
    int tail = (s_fifo_head - s_fifo_count + SIM_FIFO_CAP) % SIM_FIFO_CAP;
    for (int i = 0; i < n; i++) {
        out->samples[i] = s_fifo[(tail + i) % SIM_FIFO_CAP];
    }
    s_fifo_count -= n;
    out->count = (uint8_t)n;
    out->words_read = (uint16_t)(n * 2);
    out->truncated = s_fifo_count > 0;
    out->fifo_overrun = s_fifo_overrun;
    s_fifo_overrun = false;
    return IMU_OK;
}

float imu_sample_period_s(void)
{
    return (s_imu_cfg.odr_hz_measured > 0.0f) ? (1.0f / s_imu_cfg.odr_hz_measured) : (1.0f / 7680.0f);
}

void fcsim_imu_push(const float gyro_rps[3], const float accel_g[3])
{
    imu_sample_t *s = &s_fifo[s_fifo_head];
    s->gyro_rps.x = gyro_rps[0]; s->gyro_rps.y = gyro_rps[1]; s->gyro_rps.z = gyro_rps[2];
    s->accel_g.x  = accel_g[0];  s->accel_g.y  = accel_g[1];  s->accel_g.z  = accel_g[2];
    s_fifo_head = (s_fifo_head + 1) % SIM_FIFO_CAP;
    if (s_fifo_count < SIM_FIFO_CAP) {
        s_fifo_count++;
    } else {
        s_fifo_overrun = true;   /* 本物と同じく、いちばん古いものが上書きされる */
    }
}

int32_t fcsim_imu_pending(void) { return s_fifo_count; }

float fcsim_imu_sample_period_s(void) { return imu_sample_period_s(); }

/* ---------------- モーターPWM ---------------- */

static float s_duty[FC_NUM_MOTORS];

void motor_pwm_init(void) { memset(s_duty, 0, sizeof(s_duty)); }

void motor_pwm_set(const float duty[FC_NUM_MOTORS])
{
    for (int i = 0; i < FC_NUM_MOTORS; i++) {
        float d = duty[i];
        if (!(d > 0.0f)) d = 0.0f;   /* NaNもここで0（本物と同じ） */
        if (d > 1.0f) d = 1.0f;
        s_duty[i] = d;
    }
}

void motor_pwm_stop(void) { memset(s_duty, 0, sizeof(s_duty)); }

void fcsim_get_duty(float duty_out[4])
{
    for (int i = 0; i < FC_NUM_MOTORS; i++) duty_out[i] = s_duty[i];
}

/* ---------------- 時計・WDT ---------------- */

static uint64_t s_now_us;
static uint32_t s_wdt_feeds;
static uint32_t s_wdt_timeout_ms;

uint64_t fc_clock_now_us64(void) { return s_now_us; }
uint32_t fc_clock_now_us32(void) { return (uint32_t)s_now_us; }

void fc_wdt_start(uint32_t timeout_ms) { s_wdt_timeout_ms = timeout_ms; }
void fc_wdt_feed(void) { s_wdt_feeds++; }
bool fc_wdt_caused_reboot(void) { return false; }

void fcsim_set_time_us(uint64_t now_us)
{
    if (now_us >= s_now_us) s_now_us = now_us;
}

uint32_t fcsim_wdt_feed_count(void) { return s_wdt_feeds; }

/* fcsim.c の fcsim_init から呼ぶ（時計とWDTの記録を最初に戻す） */
void port_sim_reset_clock(void)
{
    s_now_us = 0;
    s_wdt_feeds = 0;
    s_wdt_timeout_ms = 0;
}

uint32_t fcsim_wdt_timeout_ms(void) { return s_wdt_timeout_ms; }

/* ---------------- 電池ADC ---------------- */

static uint16_t s_batt_raw;

void batt_adc_init(void) {}
uint16_t batt_adc_read_raw(void) { return s_batt_raw; }

void fcsim_set_batt_raw(uint16_t raw) { s_batt_raw = raw > 4095u ? 4095u : raw; }

void fcsim_set_batt_voltage(float volts)
{
    /* battery_update の逆: raw = V ÷ 分圧比 ÷ (基準電圧/4095)。12bitに丸めるのも本物のADCと同じ */
    const battery_params_t *p = &params_get()->battery;
    float v_adc = volts / p->adc_divider_ratio;
    float raw = v_adc * (4095.0f / p->adc_vref_v);
    if (!(raw > 0.0f)) raw = 0.0f;
    if (raw > 4095.0f) raw = 4095.0f;
    s_batt_raw = (uint16_t)(raw + 0.5f);
}
