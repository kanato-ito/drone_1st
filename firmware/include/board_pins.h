/**
 * @file board_pins.h
 * @brief 発注版基板のピン割り当て（変更不可、設計メモ1章）。SDK非依存の定数のみ。
 *
 * 実装(.c)側で pwm_gpio_to_slice_num() などの結果とここの値が一致することを
 * assert で確認すること（転記ミス防止）。
 */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/* ---- モーター PWM ---- */
#define BOARD_PWM_CH_A 0
#define BOARD_PWM_CH_B 1

#define BOARD_M1_GPIO   0   /* スライス0 A */
#define BOARD_M1_SLICE  0
#define BOARD_M1_CHAN   BOARD_PWM_CH_A

#define BOARD_M2_GPIO   15  /* スライス7 B */
#define BOARD_M2_SLICE  7
#define BOARD_M2_CHAN   BOARD_PWM_CH_B

#define BOARD_M3_GPIO   18  /* スライス1 A */
#define BOARD_M3_SLICE  1
#define BOARD_M3_CHAN   BOARD_PWM_CH_A

#define BOARD_M4_GPIO   17  /* スライス0 B（M1とスライス共通＝周波数共通） */
#define BOARD_M4_SLICE  0
#define BOARD_M4_CHAN   BOARD_PWM_CH_B

/** PWM周波数 [Hz]（設計メモ1章: 20kHz）。 */
#define BOARD_PWM_FREQ_HZ 20000u

/* ---- IMU (LSM6DSV16X, SPI0, 4線) ---- */
#define BOARD_IMU_SPI_INSTANCE 0
#define BOARD_IMU_MOSI_GPIO 7
#define BOARD_IMU_MISO_GPIO 20
#define BOARD_IMU_SCK_GPIO  22
#define BOARD_IMU_CS_GPIO   27  /* SPIペリフェラルではなくGPIOで操作する */
#define BOARD_IMU_INT1_GPIO 26  /* デジタル入力・立ち上がり割り込み（ADC0としては使わない） */

/** SPIクロックの指定値 [Hz]。実際は150MHz÷20=7.5MHz（spi_init()の戻り値で確認する）。 */
#define BOARD_IMU_SPI_BAUD_REQUEST_HZ 8000000u

/* ---- 電池電圧 ---- */
#define BOARD_BATT_GPIO      28
#define BOARD_BATT_ADC_INPUT 2  /* ADC2 */

#endif /* BOARD_PINS_H */
