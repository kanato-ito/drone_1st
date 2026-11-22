/**
 * @file test_gyro_filter.c
 * @brief ジャイロのローパス（gyro_filter.c）の試験。純粋関数なので入力の列を作って出力を見るだけ。
 */
#include <stdlib.h>
#include "gyro_filter.h"
#include "test_common.h"

#define FS_HZ 7519.0f   /* IMUの実測ODR */
#define PI_F  3.14159265f

/** 1軸(x)に値 v を8サンプル入れたバッチを作る。y・z は別の一定値。 */
static void fill_batch(imu_batch_t *b, int n, float (*fx)(int, void *), void *ctx, int k0, float y, float z)
{
    b->count = (uint8_t)n; b->words_read = (uint16_t)(2 * n); b->fifo_overrun = false; b->truncated = false;
    for (int i = 0; i < n; i++) {
        b->samples[i].gyro_rps = (vec3f_t){ fx(k0 + i, ctx), y, z };
        b->samples[i].accel_g = (vec3f_t){ 0.0f, 0.0f, 1.0f };
    }
}

static float sine_fn(int k, void *ctx)
{
    const float hz = *(const float *)ctx;
    return sinf(2.0f * PI_F * hz * (float)k / FS_HZ);
}
static float const_fn(int k, void *ctx) { (void)k; return *(const float *)ctx; }

/** hz の正弦波（振幅1）を nb バッチ入れ、後半の出力の最大絶対値（= 振幅）を返す。1サンプルごとに出力を見るため1サンプルのバッチで回す */
static float sine_gain(float cutoff_hz, float hz, int n_samples)
{
    gyro_filter_t f;
    gyro_filter_init(&f, cutoff_hz, FS_HZ);
    imu_batch_t b;
    float peak = 0.0f;
    for (int k = 0; k < n_samples; k++) {
        fill_batch(&b, 1, sine_fn, &hz, k, 0.0f, 0.0f);
        gyro_filter_apply_batch(&f, &b);
        if (k > n_samples / 2) {   /* 過渡応答（数ms）が終わった後半だけ見る */
            const float y = fabsf(gyro_filter_output(&f).x);
            if (y > peak) peak = y;
        }
    }
    return peak;
}

static void test_gyro_dc_and_seed(void)
{
    gyro_filter_t f;
    gyro_filter_init(&f, 80.0f, FS_HZ);
    /* 初期化直後の出力は0 */
    CHECK(gyro_filter_output(&f).x == 0.0f);
    float c = 1.7f;
    imu_batch_t b;
    fill_batch(&b, 8, const_fn, &c, 0, -0.3f, 0.05f);
    gyro_filter_apply_batch(&f, &b);
    /* 最初のサンプルで状態を合わせるので、一定の入力は最初から同じ値（過渡応答なし）。3軸とも */
    vec3f_t o = gyro_filter_output(&f);
    CHECK_NEAR(o.x, 1.7f, 1e-5f);
    CHECK_NEAR(o.y, -0.3f, 1e-5f);
    CHECK_NEAR(o.z, 0.05f, 1e-5f);
    /* 長く続けても同じ（直流利得1） */
    for (int i = 0; i < 400; i++) gyro_filter_apply_batch(&f, &b);
    o = gyro_filter_output(&f);
    CHECK_NEAR(o.x, 1.7f, 1e-3f);
    CHECK_NEAR(o.y, -0.3f, 1e-3f);
    CHECK_NEAR(o.z, 0.05f, 1e-3f);
    /* 入力は書き換えない */
    CHECK(b.count == 8 && b.samples[7].gyro_rps.x == 1.7f);
}

static void test_gyro_step_is_smooth(void)
{
    /* 0 に落ち着いたあと1に飛ぶ: 8サンプルでは約1割（理論: 2次バターワース 80Hz のステップ応答）。行き過ぎ（オーバーシュート）は約4% */
    gyro_filter_t f;
    gyro_filter_init(&f, 80.0f, FS_HZ);
    imu_batch_t b;
    float z = 0.0f, one = 1.0f;
    fill_batch(&b, 8, const_fn, &z, 0, 0.0f, 0.0f);
    for (int i = 0; i < 10; i++) gyro_filter_apply_batch(&f, &b);
    fill_batch(&b, 8, const_fn, &one, 0, 0.0f, 0.0f);
    gyro_filter_apply_batch(&f, &b);
    const float y8 = gyro_filter_output(&f).x;
    CHECK(y8 > 0.05f && y8 < 0.20f);
    float peak = y8;
    for (int i = 0; i < 100; i++) {
        gyro_filter_apply_batch(&f, &b);
        if (gyro_filter_output(&f).x > peak) peak = gyro_filter_output(&f).x;
    }
    CHECK(peak > 1.0f && peak < 1.08f);
    CHECK_NEAR(gyro_filter_output(&f).x, 1.0f, 1e-3f);
}

static void test_gyro_frequency_response(void)
{
    /* 240Hz: 理論値 1/√(1+(240/80)^4) = 0.1104。プリワープつき双一次変換なので、離散の理論値は
     * 1/√(1+(tan(π240/fs)/tan(π80/fs))^4) ≒ 0.1098 に近い。許容は数％ */
    const float g240 = sine_gain(80.0f, 240.0f, 4000);
    CHECK_NEAR(g240, 0.110f, 0.110f * 0.04f);
    /* 遮断周波数ではちょうど 1/√2（プリワープで合わせてある） */
    const float g80 = sine_gain(80.0f, 80.0f, 4000);
    CHECK_NEAR(g80, 0.7071f, 0.7071f * 0.02f);
    /* 20Hz はほぼ通す */
    const float g20 = sine_gain(80.0f, 20.0f, 6000);
    CHECK_NEAR(g20, 1.0f, 0.01f);
    /* 2次なので -40dB/dec: 800Hz は 1/100 に近い（約0.01 の倍までを許す） */
    CHECK(sine_gain(80.0f, 800.0f, 4000) < 0.02f);
}

static void test_gyro_off_and_invalid(void)
{
    imu_batch_t b;
    float c = 0.0f;
    const float cutoffs[] = { 0.0f, -5.0f, 0.45f * FS_HZ, 4000.0f, NAN, INFINITY };
    for (int k = 0; k < 6; k++) {
        gyro_filter_t f;
        gyro_filter_init(&f, cutoffs[k], FS_HZ);
        /* 素通し: 最後のサンプルがそのまま出る */
        for (int i = 0; i < 8; i++) {
            b.samples[i].gyro_rps = (vec3f_t){ 0.1f * (float)i, -0.2f * (float)i, 3.0f - (float)i };
            b.samples[i].accel_g = (vec3f_t){ 0, 0, 1 };
        }
        b.count = 8;
        gyro_filter_apply_batch(&f, &b);
        vec3f_t o = gyro_filter_output(&f);
        CHECK_NEAR(o.x, 0.7f, 0.0f); CHECK_NEAR(o.y, -1.4f, 0.0f); CHECK_NEAR(o.z, -4.0f, 0.0f);
        (void)c;
    }
    /* サンプル周波数が不正でも素通し */
    const float fss[] = { 0.0f, -1.0f, NAN };
    for (int k = 0; k < 3; k++) {
        gyro_filter_t f;
        gyro_filter_init(&f, 80.0f, fss[k]);
        gyro_filter_apply_batch(&f, &b);
        CHECK_NEAR(gyro_filter_output(&f).x, 0.7f, 0.0f);
    }
}

static void test_gyro_batch_order_and_empty(void)
{
    /* バッチで入れても、1サンプルずつ入れても同じ（サンプルの順に通す）。count==0 は何もしない */
    gyro_filter_t fa, fb;
    gyro_filter_init(&fa, 80.0f, FS_HZ);
    gyro_filter_init(&fb, 80.0f, FS_HZ);
    float hz = 150.0f;
    imu_batch_t big, one, empty;
    memset(&empty, 0, sizeof(empty));
    for (int n = 0; n < 50; n++) {
        fill_batch(&big, 8, sine_fn, &hz, n * 8, 0.2f, -0.1f);
        gyro_filter_apply_batch(&fa, &big);
        for (int i = 0; i < 8; i++) {
            fill_batch(&one, 1, sine_fn, &hz, n * 8 + i, 0.2f, -0.1f);
            gyro_filter_apply_batch(&fb, &one);
        }
        CHECK_NEAR(gyro_filter_output(&fa).x, gyro_filter_output(&fb).x, 1e-6f);
    }
    vec3f_t before = gyro_filter_output(&fa);
    gyro_filter_apply_batch(&fa, &empty);
    vec3f_t after = gyro_filter_output(&fa);
    CHECK(before.x == after.x && before.y == after.y && before.z == after.z);
}

const test_t g_tests_gyro[] = {
    { "gyro_filter: 直流利得1・最初のサンプルで状態を合わせる", test_gyro_dc_and_seed },
    { "gyro_filter: ステップ応答（なめらか・行き過ぎ小）", test_gyro_step_is_smooth },
    { "gyro_filter: 周波数特性（80Hzで-3dB、240Hzで約0.11、20Hzで1）", test_gyro_frequency_response },
    { "gyro_filter: 遮断0・不正な値は素通し", test_gyro_off_and_invalid },
    { "gyro_filter: バッチと1サンプルずつが同じ・count0は何もしない", test_gyro_batch_order_and_empty },
};
const int g_tests_gyro_n = (int)(sizeof(g_tests_gyro) / sizeof(g_tests_gyro[0]));
