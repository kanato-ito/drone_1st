/**
 * @file gyro_filter.c
 * @brief ジャイロのローパス（gyro_filter.h）。2次バターワース、直接形II転置。
 *
 * 係数（K = tan(π fc / fs)、Q = 1/√2）:
 *   norm = 1 / (1 + K/Q + K²)
 *   b0 = K² norm,  b1 = 2 b0,  b2 = b0
 *   a1 = 2 (K² - 1) norm,  a2 = (1 - K/Q + K²) norm
 * 直接形II転置: y = b0 x + z1;  z1 = b1 x - a1 y + z2;  z2 = b2 x - a2 y
 */
#include <math.h>
#include <string.h>
#include "gyro_filter.h"

#define PI_F 3.14159265358979f

void gyro_filter_init(gyro_filter_t *f, float cutoff_hz, float sample_hz)
{
    memset(f, 0, sizeof(*f));
    /* NaN は比較がすべて false になるので、「有効な範囲に入っている」形で書く */
    const bool ok = isfinite(cutoff_hz) && isfinite(sample_hz) && cutoff_hz > 0.0f && sample_hz > 0.0f &&
                    cutoff_hz < 0.45f * sample_hz;
    if (!ok) {
        f->enabled = false;
        return;
    }
    /* 係数はdoubleで作る（遮断周波数がサンプル周波数の1%ほどだと、float では 1+a1+a2 の桁落ちが大きい）。起動時1回だけ */
    const double k = tan((double)PI_F * (double)cutoff_hz / (double)sample_hz);
    const double q = 0.70710678118654752;   /* 1/√2 */
    const double norm = 1.0 / (1.0 + k / q + k * k);
    f->b0 = (float)(k * k * norm);
    f->b1 = 2.0f * f->b0;
    f->b2 = f->b0;
    f->a1 = (float)(2.0 * (k * k - 1.0) * norm);
    f->a2 = (float)((1.0 - k / q + k * k) * norm);
    f->enabled = true;
}

/** 状態を「入力が x でずっと続いていた」ときの値にする。最初の出力が x になる。 */
static void seed(gyro_filter_t *f, int axis, float x)
{
    f->z2[axis] = (f->b2 - f->a2) * x;
    f->z1[axis] = x - f->b0 * x;   /* 最初の出力 b0 x + z1 が x ちょうどになる */
}

void gyro_filter_apply_batch(gyro_filter_t *f, const imu_batch_t *batch)
{
    for (int i = 0; i < (int)batch->count; i++) {
        const float in[3] = { batch->samples[i].gyro_rps.x, batch->samples[i].gyro_rps.y, batch->samples[i].gyro_rps.z };
        float out[3];
        if (!f->enabled) {
            memcpy(out, in, sizeof(out));
        } else {
            if (!f->seeded) {
                for (int a = 0; a < 3; a++) seed(f, a, in[a]);
                f->seeded = true;
            }
            for (int a = 0; a < 3; a++) {
                const float y = f->b0 * in[a] + f->z1[a];
                f->z1[a] = f->b1 * in[a] - f->a1 * y + f->z2[a];
                f->z2[a] = f->b2 * in[a] - f->a2 * y;
                out[a] = y;
            }
        }
        f->out = (vec3f_t){ out[0], out[1], out[2] };
    }
}

vec3f_t gyro_filter_output(const gyro_filter_t *f)
{
    return f->out;
}
