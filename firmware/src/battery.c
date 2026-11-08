/**
 * @file battery.c
 * @brief 電池電圧（GP28 = ADC2）の計測とローパス（battery.h）。
 */
#include <math.h>
#include "hardware/adc.h"
#include "board_pins.h"
#include "battery.h"

/* GP28 = ADC2 の対応を確認（転記ミス防止）。RP2350A は GP26..29 が ADC0..3 */
_Static_assert(BOARD_BATT_GPIO - 26 == BOARD_BATT_ADC_INPUT, "電池電圧のGPIOとADC入力が一致しない");

void battery_init(battery_state_t *b)
{
    b->voltage_v = 0.0f; b->millivolt = 0; b->low = false; b->below_since_ms = 0;
    b->last_ms = 0; b->raw_v = 0.0f; b->primed = false; b->below = false;
    adc_init();
    adc_gpio_init(BOARD_BATT_GPIO);
    adc_select_input(BOARD_BATT_ADC_INPUT);
}

void battery_apply_sample(battery_state_t *b, const battery_params_t *p, float v_raw, uint32_t now_ms)
{
    b->raw_v = v_raw;
    if (!b->primed) {
        b->voltage_v = v_raw;   /* 最初の1回はそのまま採用（0Vから立ち上げると低下と誤判定するため） */
        b->primed = true;
    } else {
        float dt = (float)(uint32_t)(now_ms - b->last_ms) * 1.0e-3f;
        b->voltage_v += (dt / (p->lpf_tau_s + dt)) * (v_raw - b->voltage_v);
    }
    b->last_ms = now_ms;
    b->millivolt = (uint16_t)(b->voltage_v < 0.0f ? 0.0f : (b->voltage_v > 65.0f ? 65000.0f : b->voltage_v * 1000.0f));

    if (b->voltage_v < p->v_low) {
        if (!b->below) { b->below = true; b->below_since_ms = now_ms; }
        if ((uint32_t)(now_ms - b->below_since_ms) >= p->low_hold_ms) b->low = true;
    } else {
        b->below = false;
        b->low = false;
    }
}

void battery_update(battery_state_t *b, const battery_params_t *p, uint32_t now_ms)
{
    uint16_t raw = adc_read();   /* 12bit。adc_select_input は init で済み */
    float v_adc = (float)raw * (p->adc_vref_v / 4095.0f);
    battery_apply_sample(b, p, v_adc * p->adc_divider_ratio, now_ms);
}
