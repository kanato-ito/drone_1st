/** @file fake_port.h  fake_port.c の偽物を操作する変数。 */
#ifndef FAKE_PORT_H
#define FAKE_PORT_H

#include <stdint.h>
#include <stdbool.h>
#include "fc_types.h"
#include "imu_lsm6dsv16x.h"

extern uint16_t fake_batt_raw;        /**< batt_adc_read_raw が返す12bitカウント */
extern uint32_t fake_now_us;          /**< fc_clock_now_us32/64 が返す時刻 [µs] */
extern int      fake_batt_init_calls; /**< batt_adc_init が呼ばれた回数 */

/* ---- WDT ---- */
extern int      fake_wdt_start_calls;
extern uint32_t fake_wdt_timeout_ms;  /**< fc_wdt_start に渡された値 */
extern uint32_t fake_wdt_feeds;       /**< fc_wdt_feed の回数 */
extern bool     fake_wdt_reboot;      /**< fc_wdt_caused_reboot が返す値 */

/* ---- モーターPWM ---- */
extern int   fake_pwm_init_calls;
extern int   fake_pwm_set_calls;
extern int   fake_pwm_stop_calls;
extern float fake_pwm_duty[FC_NUM_MOTORS]; /**< いま出ているデューティ（stop で全部0） */

/* ---- IMU（基板の軸のまま値を作る。並べ替えは control_loop 側）---- */
extern imu_status_t fake_imu_init_result;
extern imu_status_t fake_imu_read_result;
extern vec3f_t fake_imu_gyro;          /**< 基板の軸 [rad/s] */
extern vec3f_t fake_imu_accel;         /**< 基板の軸 [g] */
extern bool     fake_imu_wait_timeout; /**< true の間 imu_wait_int1 は5ms進めて false を返す */
extern bool     fake_imu_pin_high;     /**< imu_int1_pin_is_high の値 */
extern uint32_t fake_imu_period_us;    /**< INT1の間隔（1064） */
extern uint32_t fake_imu_read_cost_us; /**< imu_read_fifo が進める時間（proc_us の確認用） */
extern uint32_t fake_imu_int1_count;
extern int      fake_imu_wait_calls;
extern int      fake_imu_read_calls;

/** 偽物の状態を全部初期値に戻す（各試験の最初に呼ぶ）。時刻は 1,000,000µs から始める。 */
void fake_port_reset(void);

/** 機体座標（x前・y左・z上）の加速度 [g] を、基板の軸（axis_map {2,-1,3} の逆変換）で fake_imu_accel に入れる。 */
void fake_imu_set_body_accel(float bx, float by, float bz);
void fake_imu_set_body_gyro(float bx, float by, float bz);

#endif /* FAKE_PORT_H */
