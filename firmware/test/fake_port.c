/**
 * @file fake_port.c
 * @brief ポート（差し替え境界）の偽物。ホストPCの単体試験用。シミュレータ班の見本にもなる。
 *
 * 実機は src/port_pico/ の各.c が本物を実装している。ここでは同じヘッダの関数を、SDKなしで
 * 「値をセットして返す」だけの形で実装する。試験は fake_* の変数を書き換えて使う。
 * 対象: batt_adc, 時計, WDT, motor_pwm, IMU（制御周期まるごとの試験用。③bで追加）。
 */
#include "fake_port.h"
#include "batt_adc.h"
#include "cycle_timer.h"
#include "motor_pwm.h"
#include "imu_lsm6dsv16x.h"

uint16_t fake_batt_raw;
uint32_t fake_now_us;
int      fake_batt_init_calls;

void batt_adc_init(void) { fake_batt_init_calls++; }
uint16_t batt_adc_read_raw(void) { return fake_batt_raw; }

uint64_t fc_clock_now_us64(void) { return fake_now_us; }
uint32_t fc_clock_now_us32(void) { return fake_now_us; }

/* ---- WDT ---- */
int      fake_wdt_start_calls;
uint32_t fake_wdt_timeout_ms;
uint32_t fake_wdt_feeds;
bool     fake_wdt_reboot;

void fc_wdt_start(uint32_t timeout_ms) { fake_wdt_start_calls++; fake_wdt_timeout_ms = timeout_ms; }
void fc_wdt_feed(void) { fake_wdt_feeds++; }
bool fc_wdt_caused_reboot(void) { return fake_wdt_reboot; }

/* ---- モーターPWM ---- */
int   fake_pwm_init_calls, fake_pwm_set_calls, fake_pwm_stop_calls;
float fake_pwm_duty[FC_NUM_MOTORS];

void motor_pwm_init(void)
{
    fake_pwm_init_calls++;
    for (int i = 0; i < FC_NUM_MOTORS; i++) fake_pwm_duty[i] = 0.0f;
}

void motor_pwm_set(const float duty[FC_NUM_MOTORS])
{
    fake_pwm_set_calls++;
    for (int i = 0; i < FC_NUM_MOTORS; i++) {
        float d = duty[i];
        fake_pwm_duty[i] = (d >= 0.0f && d <= 1.0f) ? d : 0.0f;   /* 範囲外・NaNは0（本物と同じ） */
    }
}

void motor_pwm_stop(void)
{
    fake_pwm_stop_calls++;
    for (int i = 0; i < FC_NUM_MOTORS; i++) fake_pwm_duty[i] = 0.0f;
}

/* ---- IMU ---- */
imu_status_t fake_imu_init_result;
imu_status_t fake_imu_read_result;
vec3f_t fake_imu_gyro, fake_imu_accel;
bool     fake_imu_wait_timeout, fake_imu_pin_high;
uint32_t fake_imu_period_us, fake_imu_read_cost_us, fake_imu_int1_count;
int      fake_imu_wait_calls, fake_imu_read_calls;
int      fake_imu_read_samples;

imu_config_t imu_config_default(void)
{
    imu_config_t c = { 8000000u, 16, 7519.0f, 2000, 16 };
    return c;
}

imu_status_t imu_init(const imu_config_t *cfg) { (void)cfg; return fake_imu_init_result; }
float imu_sample_period_s(void) { return 1.0f / 7519.0f; }
bool imu_int1_pin_is_high(void) { return fake_imu_pin_high; }

bool imu_wait_int1(uint32_t timeout_us, imu_int1_event_t *ev)
{
    fake_imu_wait_calls++;
    if (fake_imu_wait_timeout) {
        fake_now_us += timeout_us;
        return false;
    }
    fake_now_us += fake_imu_period_us;
    fake_imu_int1_count++;
    ev->count = fake_imu_int1_count;
    ev->timestamp_us = fake_now_us;
    return true;
}

imu_status_t imu_read_fifo(imu_batch_t *out)
{
    fake_imu_read_calls++;
    fake_now_us += fake_imu_read_cost_us;
    if (fake_imu_read_result != IMU_OK) return fake_imu_read_result;
    out->count = (uint8_t)fake_imu_read_samples;
    out->words_read = (uint16_t)(2 * fake_imu_read_samples);
    out->fifo_overrun = false;
    out->truncated = false;
    for (int i = 0; i < fake_imu_read_samples; i++) {
        out->samples[i].gyro_rps = fake_imu_gyro;
        out->samples[i].accel_g = fake_imu_accel;
    }
    return IMU_OK;
}

void fake_port_reset(void)
{
    fake_batt_raw = 2358;               /* 3.8V（分圧比2.0, 3.3V基準） */
    fake_now_us = 1000000u;
    fake_batt_init_calls = 0;
    fake_wdt_start_calls = 0; fake_wdt_timeout_ms = 0; fake_wdt_feeds = 0; fake_wdt_reboot = false;
    fake_pwm_init_calls = fake_pwm_set_calls = fake_pwm_stop_calls = 0;
    for (int i = 0; i < FC_NUM_MOTORS; i++) fake_pwm_duty[i] = 0.0f;
    fake_imu_init_result = IMU_OK;
    fake_imu_read_result = IMU_OK;
    fake_imu_gyro = (vec3f_t){ 0.0f, 0.0f, 0.0f };
    fake_imu_accel = (vec3f_t){ 0.0f, 0.0f, 1.0f };
    fake_imu_wait_timeout = false;
    fake_imu_pin_high = false;
    fake_imu_period_us = 1064;
    fake_imu_read_cost_us = 160;
    fake_imu_int1_count = 0;
    fake_imu_wait_calls = fake_imu_read_calls = 0;
    fake_imu_read_samples = 8;
}

/* 機体 = (基板y, -基板x, 基板z) なので、基板 = (-機体y, 機体x, 機体z) */
void fake_imu_set_body_accel(float bx, float by, float bz)
{
    fake_imu_accel = (vec3f_t){ -by, bx, bz };
}

void fake_imu_set_body_gyro(float bx, float by, float bz)
{
    fake_imu_gyro = (vec3f_t){ -by, bx, bz };
}
