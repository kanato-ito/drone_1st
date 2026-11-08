/**
 * @file thrust_model.c
 * @brief 推力[g] ↔ デューティ（thrust_model.h）。
 */
#include <math.h>
#include "thrust_model.h"

#define V_CALC_MAX 4.5f   /* 1セルの満充電(4.2V)に余裕を足した上限。分圧比の誤りで大きい値が来たとき用（提案） */

float thrust_model_clamp_voltage(const airframe_params_t *af, float v)
{
    /* 下限より低い値・NaN は下限に丸める: 0割りとデューティ暴走を防ぐ。ただし実電圧より低く見る分、
     * デューティは大きめ（3.8Vの実機で約27%増）になる。ADC未接続や分圧比の誤りは電池低下検出（③b）と
     * 試験で見つける前提。ブレッドボードではPWMを既定で無効にしてある。 */
    if (!(v > af->voltage_min_calc_v)) return af->voltage_min_calc_v;
    if (v > V_CALC_MAX) return V_CALC_MAX;
    return v;
}

float thrust_model_max_g(const airframe_params_t *af, float battery_v)
{
    return af->thrust_k_g_per_vduty * thrust_model_clamp_voltage(af, battery_v);
}

float thrust_model_to_duty(const airframe_params_t *af, float thrust_g, float battery_v)
{
    if (!isfinite(thrust_g) || thrust_g <= 0.0f) return 0.0f;
    float d = thrust_g / thrust_model_max_g(af, battery_v);
    return d > 1.0f ? 1.0f : d;
}
