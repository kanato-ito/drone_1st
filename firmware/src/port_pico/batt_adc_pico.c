/**
 * @file batt_adc_pico.c
 * @brief batt_adc.h の実装（ポート: Pico SDK 版）。GP28 = ADC2。
 */
#include "hardware/adc.h"
#include "board_pins.h"
#include "batt_adc.h"

/* GP28 = ADC2 の対応を確認（転記ミス防止）。RP2350A は GP26..29 が ADC0..3 */
_Static_assert(BOARD_BATT_GPIO - 26 == BOARD_BATT_ADC_INPUT, "電池電圧のGPIOとADC入力が一致しない");

void batt_adc_init(void)
{
    adc_init();
    adc_gpio_init(BOARD_BATT_GPIO);
    adc_select_input(BOARD_BATT_ADC_INPUT);
}

uint16_t batt_adc_read_raw(void)
{
    return adc_read();   /* 12bit。adc_select_input は init で済み */
}
