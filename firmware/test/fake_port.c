/**
 * @file fake_port.c
 * @brief ポート（差し替え境界）の偽物。ホストPCの単体試験用。シミュレータ班の見本にもなる。
 *
 * 実機は src/port_pico/ の各.c が本物を実装している。ここでは同じヘッダの関数を、SDKなしで
 * 「値をセットして返す」だけの形で実装する。試験は fake_* の変数を書き換えて使う。
 * （IMU・モーターPWM・WDTの偽物は、制御周期まるごとの試験を作るとき（ステップ③b）に足す）
 */
#include "fake_port.h"
#include "batt_adc.h"
#include "cycle_timer.h"

uint16_t fake_batt_raw;
uint32_t fake_now_us;
int      fake_batt_init_calls;

void batt_adc_init(void) { fake_batt_init_calls++; }
uint16_t batt_adc_read_raw(void) { return fake_batt_raw; }

uint64_t fc_clock_now_us64(void) { return fake_now_us; }
uint32_t fc_clock_now_us32(void) { return fake_now_us; }

void fc_wdt_start(uint32_t timeout_ms) { (void)timeout_ms; }
void fc_wdt_feed(void) {}
bool fc_wdt_caused_reboot(void) { return false; }
