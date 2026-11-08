/** @file fake_port.h  fake_port.c の偽物を操作する変数。 */
#ifndef FAKE_PORT_H
#define FAKE_PORT_H

#include <stdint.h>

extern uint16_t fake_batt_raw;        /**< batt_adc_read_raw が返す12bitカウント */
extern uint32_t fake_now_us;          /**< fc_clock_now_us32/64 が返す時刻 [µs] */
extern int      fake_batt_init_calls; /**< batt_adc_init が呼ばれた回数 */

#endif /* FAKE_PORT_H */
