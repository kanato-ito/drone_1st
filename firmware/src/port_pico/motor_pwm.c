/**
 * @file motor_pwm.c
 * @brief モーターPWM 20kHz（motor_pwm.h）。
 *
 * 分周1・wrap = clk_sys/20kHz - 1（150MHzで7499）。M1/M4はスライス0のA/Bで周波数が共通。
 * デューティ0は常にLow。モーター駆動ゲートが反転かどうかは未確認（architecture.md #24）。
 */
#include <math.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "board_pins.h"
#include "motor_pwm.h"

typedef struct { uint gpio; uint slice; uint chan; } motor_pin_t;

static const motor_pin_t s_pins[FC_NUM_MOTORS] = {
    { BOARD_M1_GPIO, BOARD_M1_SLICE, BOARD_M1_CHAN },
    { BOARD_M2_GPIO, BOARD_M2_SLICE, BOARD_M2_CHAN },
    { BOARD_M3_GPIO, BOARD_M3_SLICE, BOARD_M3_CHAN },
    { BOARD_M4_GPIO, BOARD_M4_SLICE, BOARD_M4_CHAN },
};

static uint16_t s_top;   /* wrap値。level の最大は s_top + 1（100%） */

void motor_pwm_init(void)
{
    uint32_t clk = clock_get_hz(clk_sys);
    uint32_t counts = clk / BOARD_PWM_FREQ_HZ;       /* 150MHz/20kHz = 7500 */
    s_top = (uint16_t)(counts - 1u);                  /* 65536未満なので分周1で足りる */

    for (int i = 0; i < FC_NUM_MOTORS; i++) {
        /* 転記ミス防止: board_pins.h の表とSDKの計算が一致すること */
        if (pwm_gpio_to_slice_num(s_pins[i].gpio) != s_pins[i].slice ||
            pwm_gpio_to_channel(s_pins[i].gpio) != s_pins[i].chan) {
            for (;;) { tight_loop_contents(); }   /* 配線表の誤り。出力を出さず止まる */
        }
        gpio_set_function(s_pins[i].gpio, GPIO_FUNC_PWM);
        pwm_set_chan_level(s_pins[i].slice, s_pins[i].chan, 0);
    }
    /* スライスは0,1,7の3つ。M1/M4は同じスライスなので1回だけ設定 */
    static const uint slices[3] = { BOARD_M1_SLICE, BOARD_M3_SLICE, BOARD_M2_SLICE };
    for (int i = 0; i < 3; i++) {
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_config_set_wrap(&cfg, s_top);
        pwm_init(slices[i], &cfg, true);
        pwm_set_chan_level(slices[i], 0, 0);
        pwm_set_chan_level(slices[i], 1, 0);
    }
}

void motor_pwm_set(const float duty[FC_NUM_MOTORS])
{
    for (int i = 0; i < FC_NUM_MOTORS; i++) {
        float d = duty[i];
        if (!(d > 0.0f)) d = 0.0f;       /* NaNもここで0になる */
        if (d > 1.0f) d = 1.0f;
        uint16_t level = (uint16_t)(d * (float)(s_top + 1u) + 0.5f);
        pwm_set_chan_level(s_pins[i].slice, s_pins[i].chan, level);
    }
}

void motor_pwm_stop(void)
{
    for (int i = 0; i < FC_NUM_MOTORS; i++) {
        pwm_set_chan_level(s_pins[i].slice, s_pins[i].chan, 0);
    }
}
