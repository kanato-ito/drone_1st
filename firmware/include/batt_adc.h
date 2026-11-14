/**
 * @file batt_adc.h
 * @brief 電池電圧のADC読み取り（GP28 = ADC2）。生のカウントを返すだけの薄い層。SDK非依存。
 *
 * ポート（差し替え境界）: 実機は port_pico ディレクトリの batt_adc_pico.c、シミュレータは同じ関数を自前の.cで実装する。
 * 分圧比・基準電圧による電圧への換算は battery.c（純粋C）が params で行う。ここでは行わない。
 */
#ifndef BATT_ADC_H
#define BATT_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** ADCとGP28を初期化し、入力をADC2に選ぶ。呼ぶコア: コア0（battery_init から）。頻度: 起動時に1回。 */
void batt_adc_init(void);

/**
 * GP28/ADC2 を1回変換して、12bitの生のカウント(0..4095)を返す。換算前の値。
 * 呼ぶコア: コア0（battery_update から）。頻度: 数周期に1回。所要: 数µs（ブロッキング）。
 */
uint16_t batt_adc_read_raw(void);

#ifdef __cplusplus
}
#endif

#endif /* BATT_ADC_H */
