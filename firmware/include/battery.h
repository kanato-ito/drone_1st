/**
 * @file battery.h
 * @brief 電池電圧（GP28 = ADC2）の計測と低下判定。
 *
 * コア0。電池低下は非常時降下のきっかけになる（設計メモ5章）ので安全系と同じコアに置く
 * （提案。メモは担当コアを書いていない）。
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>
#include "params.h"

typedef struct {
    float    voltage_v;      /**< ローパス後の電圧 */
    uint16_t millivolt;      /**< ログ用 */
    bool     low;            /**< v_low 未満が low_hold_ms 続いた */
    uint32_t below_since_ms; /**< 内部 */
    uint32_t last_ms;        /**< 内部: 前回の更新時刻（ローパスのdt用） */
    float    raw_v;          /**< 直近のローパス前の電圧 [V]（表示用） */
    bool     primed;         /**< 内部: 最初の1回でローパスを初期化済み */
    bool     below;          /**< 内部: v_low 未満の最中 */
} battery_state_t;

/** ADC初期化。コア0、起動時に1回。 */
void battery_init(battery_state_t *b);

/**
 * ADC電圧に換算済みの電池電圧 v_raw を、ゆっくりしたローパス(lpf_tau_s)に通して状態を更新する。
 * battery_update の中身（ADCに触らない部分）。SDK無しで試験できるよう分けた。
 */
void battery_apply_sample(battery_state_t *b, const battery_params_t *p, float v_raw, uint32_t now_ms);

/**
 * ADCを1回読んで状態を更新する（変換は数µs）。
 * コア0。頻度: 提案として数周期に1回（電圧は遅い）。
 */
void battery_update(battery_state_t *b, const battery_params_t *p, uint32_t now_ms);

#endif /* BATTERY_H */
