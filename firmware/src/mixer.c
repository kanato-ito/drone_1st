/**
 * @file mixer.c
 * @brief ミキサー（mixer.h）。符号表で配り、はみ出したらヨー→上下ずらし→ロール・ピッチ縮小の順で収める。
 */
#include <math.h>
#include "mixer.h"

bool mixer_sign_table_valid(const int8_t s[FC_NUM_MOTORS][3])
{
    for (int m = 0; m < FC_NUM_MOTORS; m++)
        for (int a = 0; a < 3; a++)
            if (s[m][a] != 1 && s[m][a] != -1) return false;
    /* 列の和が0 = その軸の指令で全体推力が変わらない。列同士の内積0 = 軸が干渉しない。
     * ヨーの列が「対角2つ同符号」なら、ロールとピッチの列との内積も0になる */
    for (int a = 0; a < 3; a++) {
        int sum = 0;
        for (int m = 0; m < FC_NUM_MOTORS; m++) sum += s[m][a];
        if (sum != 0) return false;
    }
    for (int a = 0; a < 3; a++)
        for (int b = a + 1; b < 3; b++) {
            int dot = 0;
            for (int m = 0; m < FC_NUM_MOTORS; m++) dot += s[m][a] * s[m][b];
            if (dot != 0) return false;
        }
    return true;
}

static bool all_fit(const float m[FC_NUM_MOTORS], float max_g)
{
    for (int i = 0; i < FC_NUM_MOTORS; i++)
        if (!(m[i] >= 0.0f && m[i] <= max_g)) return false;
    return true;
}

static void min_max(const float m[FC_NUM_MOTORS], float *lo, float *hi)
{
    *lo = *hi = m[0];
    for (int i = 1; i < FC_NUM_MOTORS; i++) {
        if (m[i] < *lo) *lo = m[i];
        if (m[i] > *hi) *hi = m[i];
    }
}

void mixer_mix(const airframe_params_t *af, const controller_output_t *u,
               float max_g, mixer_result_t *out)
{
    out->yaw_scale = 1.0f; out->shift_g = 0.0f; out->rp_scale = 1.0f; out->saturated = false;
    for (int i = 0; i < FC_NUM_MOTORS; i++) out->motor_g[i] = 0.0f;

    if (!(max_g > 0.0f) || !isfinite(max_g) || !isfinite(u->thrust_g) || !isfinite(u->torque_g[0]) ||
        !isfinite(u->torque_g[1]) || !isfinite(u->torque_g[2])) {
        out->saturated = true;   /* 異常な入力は全モーター0 */
        return;
    }

    /* 全体推力は 0..max に収めてから配る（収められない分は「全体推力を犠牲に」の側で扱う） */
    float T = u->thrust_g;
    if (T < 0.0f) T = 0.0f;
    if (T > max_g) T = max_g;

    float rp[FC_NUM_MOTORS], yw[FC_NUM_MOTORS], m[FC_NUM_MOTORS];
    for (int i = 0; i < FC_NUM_MOTORS; i++) {
        rp[i] = T + af->mixer_sign[i][0] * u->torque_g[0] + af->mixer_sign[i][1] * u->torque_g[1];
        yw[i] = af->mixer_sign[i][2] * u->torque_g[2];
        m[i] = rp[i] + yw[i];
    }

    if (!all_fit(m, max_g)) {
        if (all_fit(rp, max_g)) {
            /* 手順1: ヨーだけを削れば収まる。収まる最大の割合 k を求める（モーターごとの上限・下限から） */
            float k = 1.0f;
            for (int i = 0; i < FC_NUM_MOTORS; i++) {
                if (yw[i] > 0.0f) {
                    float b = (max_g - rp[i]) / yw[i];
                    if (b < k) k = b;
                } else if (yw[i] < 0.0f) {
                    float b = (0.0f - rp[i]) / yw[i];
                    if (b < k) k = b;
                }
            }
            if (k < 0.0f) k = 0.0f;
            out->yaw_scale = k;
            for (int i = 0; i < FC_NUM_MOTORS; i++) m[i] = rp[i] + k * yw[i];
        } else {
            /* ロール・ピッチだけでもはみ出す: ヨーは全部削る（ヨーは真っ先に犠牲にする） */
            out->yaw_scale = 0.0f;
            for (int i = 0; i < FC_NUM_MOTORS; i++) m[i] = rp[i];
            float lo, hi;
            min_max(m, &lo, &hi);
            /* 手順3: モーター同士の差が max より大きい → ロール・ピッチを縮める（比率は保つ） */
            if (hi - lo > max_g) {
                float c = max_g / (hi - lo);
                out->rp_scale = c;
                for (int i = 0; i < FC_NUM_MOTORS; i++) m[i] = T + c * (rp[i] - T);
                min_max(m, &lo, &hi);
            }
            /* 手順2: 4つを同じだけ上下にずらす（全体推力を犠牲に） */
            float s = 0.0f;
            if (lo < 0.0f) s = -lo;
            else if (hi > max_g) s = max_g - hi;
            out->shift_g = s;
            for (int i = 0; i < FC_NUM_MOTORS; i++) m[i] += s;
        }
        out->saturated = true;
    }

    /* 浮動小数の丸め誤差で 0..max から僅かに外れても出力は必ず範囲内にする */
    for (int i = 0; i < FC_NUM_MOTORS; i++) {
        out->motor_g[i] = m[i] < 0.0f ? 0.0f : (m[i] > max_g ? max_g : m[i]);
    }
}
