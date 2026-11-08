/**
 * @file imu_lsm6dsv16x.c
 * @brief LSM6DSV16X の SPI ドライバ（ステップ②: FIFO 読み出し）。
 *
 * ST公式ドライバ (lsm6dsv16x-pid の lsm6dsv16x_reg.h / .c) はリポジトリに取り込まず、
 * 必要なレジスタだけを自前の小さな SPI 関数で読み書きする（学習のため）。
 * 各レジスタの出典は「lsm6dsv16x_reg.h の XXX」の形で書いてある。
 * ビット位置は同ヘッダの構造体の DRV_LITTLE_ENDIAN 側（先頭のメンバーが bit0）から読んだ。
 *
 * SPI は モード0 (CPOL0/CPHA0)、MSBファースト。先頭バイトの bit7 が 1 で読み出し、0 で書き込み。
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "board_pins.h"
#include "fc_types.h"
#include "cycle_timer.h"
#include "imu_lsm6dsv16x.h"

/* ====================================================================== */
/* ピンとSPIペリフェラルの対応チェック（board_pins.h の転記ミス防止）        */
/* ====================================================================== */
/* RP2350 の SPI は GPIO 番号で役割が決まる: (gpio/8)が偶数ならSPI0、
 * gpio%4 が 0=RX, 1=CSn, 2=SCK, 3=TX。基板の GP7/GP20/GP22 が全部SPI0の正しい役割か確認する。 */
_Static_assert(BOARD_IMU_SPI_INSTANCE == 0, "IMU は SPI0 を使う前提");
_Static_assert(((BOARD_IMU_MOSI_GPIO / 8) % 2) == 0 && (BOARD_IMU_MOSI_GPIO % 4) == 3, "MOSI は SPI0 TX のピンでない");
_Static_assert(((BOARD_IMU_MISO_GPIO / 8) % 2) == 0 && (BOARD_IMU_MISO_GPIO % 4) == 0, "MISO は SPI0 RX のピンでない");
_Static_assert(((BOARD_IMU_SCK_GPIO  / 8) % 2) == 0 && (BOARD_IMU_SCK_GPIO  % 4) == 2, "SCK は SPI0 SCK のピンでない");

#define IMU_SPI spi0

/* ====================================================================== */
/* レジスタ定義（すべて lsm6dsv16x_reg.h より）                              */
/* ====================================================================== */

/* lsm6dsv16x_reg.h の LSM6DSV16X_WHO_AM_I (0x0F)、期待値は同ヘッダの LSM6DSV16X_ID (0x70) */
#define REG_WHO_AM_I       0x0F
#define WHO_AM_I_VALUE     0x70

/* lsm6dsv16x_reg.h の LSM6DSV16X_CTRL1 (0x10): ODR_XL[3:0], OP_MODE_XL[6:4]
 * lsm6dsv16x_reg.h の LSM6DSV16X_CTRL2 (0x11): ODR_G[3:0],  OP_MODE_G[6:4] */
#define REG_CTRL1          0x10
#define REG_CTRL2          0x11
/* 同ヘッダの lsm6dsv16x_data_rate_t: LSM6DSV16X_ODR_AT_7680Hz = 0xC（0x10未満なので HAODR_CFG は触らない） */
#define ODR_7680HZ         0x0C
/* 同ヘッダの lsm6dsv16x_xl_mode_t / lsm6dsv16x_gy_mode_t: *_HIGH_PERFORMANCE_MD = 0x0 */
#define OP_MODE_HIGH_PERF  0x00

/* lsm6dsv16x_reg.h の LSM6DSV16X_CTRL3 (0x12): SW_RESET=bit0, IF_INC=bit2, BDU=bit6, BOOT=bit7 */
#define REG_CTRL3          0x12
#define CTRL3_SW_RESET     (1u << 0)
#define CTRL3_IF_INC       (1u << 2)  /* 複数バイト読みでアドレスが自動で進む（リセット後の既定も1） */
#define CTRL3_BDU          (1u << 6)  /* 上位・下位バイトを読み終わるまで出力レジスタを更新しない */

/* lsm6dsv16x_reg.h の LSM6DSV16X_CTRL6 (0x15): FS_G[3:0], LPF1_G_BW[6:4]
 * 値は lsm6dsv16x_gy_full_scale_t: 125dps=0x0, 250=0x1, 500=0x2, 1000=0x3, 2000=0x4, 4000=0xC */
#define REG_CTRL6          0x15

/* lsm6dsv16x_reg.h の LSM6DSV16X_CTRL8 (0x17): FS_XL[1:0], HP_LPF2_XL_BW[7:5]
 * 値は lsm6dsv16x_xl_full_scale_t: 2g=0x0, 4g=0x1, 8g=0x2, 16g=0x3 */
#define REG_CTRL8          0x17

/* lsm6dsv16x_reg.h の LSM6DSV16X_FIFO_CTRL1 (0x07): WTM[7:0]（単位はワード = TAG+6バイト。8bit なので最大255） */
#define REG_FIFO_CTRL1     0x07
/* lsm6dsv16x_reg.h の LSM6DSV16X_FIFO_CTRL2 (0x08): STOP_ON_WTM=bit7, FIFO_COMPR_RT_EN=bit6 など。全部0（圧縮なし・WTMで止めない） */
#define REG_FIFO_CTRL2     0x08
/* lsm6dsv16x_reg.h の LSM6DSV16X_FIFO_CTRL3 (0x09): BDR_XL[3:0], BDR_GY[7:4]
 * 値は lsm6dsv16x_fifo_xl_batch_t / lsm6dsv16x_fifo_gy_batch_t: *_BATCHED_AT_7680Hz = 0xC */
#define REG_FIFO_CTRL3     0x09
#define BDR_7680HZ         0x0C
/* lsm6dsv16x_reg.h の LSM6DSV16X_FIFO_CTRL4 (0x0A): FIFO_MODE[2:0], ODR_T_BATCH[5:4], DEC_TS_BATCH[7:6]
 * FIFO_MODE は lsm6dsv16x_fifo_mode_t: BYPASS_MODE=0x0, STREAM_MODE=0x6（= Continuous）。
 * タイムスタンプ・温度のバッチは0（FIFOに入れない）。 */
#define REG_FIFO_CTRL4     0x0A
#define FIFO_MODE_BYPASS   0x00
#define FIFO_MODE_STREAM   0x06

/* lsm6dsv16x_reg.h の LSM6DSV16X_INT1_CTRL (0x0D): INT1_FIFO_TH = bit3（ウォーターマーク到達でINT1）
 * ほか DRDY_XL=bit0, DRDY_G=bit1, FIFO_OVR=bit4, FIFO_FULL=bit5 は使わない。 */
#define REG_INT1_CTRL      0x0D
#define INT1_CTRL_FIFO_TH  (1u << 3)

/* lsm6dsv16x_reg.h の LSM6DSV16X_FIFO_STATUS1 (0x1B): DIFF_FIFO[7:0]
 * lsm6dsv16x_reg.h の LSM6DSV16X_FIFO_STATUS2 (0x1C): DIFF_FIFO[8]=bit0, FIFO_OVR_LATCHED=bit3,
 *                                                    FIFO_FULL_IA=bit5, FIFO_OVR_IA=bit6, FIFO_WTM_IA=bit7
 * 2つは連続アドレスなので1回の2バイト読みで取れる（ドライバの lsm6dsv16x_fifo_status_get も同じ）。 */
#define REG_FIFO_STATUS1   0x1B
#define FIFO_STATUS2_DIFF8     (1u << 0)
#define FIFO_STATUS2_OVR_LATCH (1u << 3)
#define FIFO_STATUS2_OVR_IA    (1u << 6)

/* lsm6dsv16x_reg.h の LSM6DSV16X_FIFO_DATA_OUT_TAG (0x78): TAG_CNT[2:1], TAG_SENSOR[7:3]
 * 続く 0x79..0x7E が X_L,X_H,Y_L,Y_H,Z_L,Z_H（LSM6DSV16X_FIFO_DATA_OUT_X_L ... Z_H）。
 * 1ワード = TAG 1バイト + データ6バイト = 7バイト（ドライバの lsm6dsv16x_fifo_out_raw_get も7バイト読み）。 */
#define REG_FIFO_DATA_OUT_TAG  0x78
#define FIFO_WORD_BYTES        7
/* lsm6dsv16x_reg.h の lsm6dsv16x_fifo_tag: LSM6DSV16X_GY_NC_TAG = 0x1, LSM6DSV16X_XL_NC_TAG = 0x2 */
#define TAG_GYRO           0x01
#define TAG_ACCEL          0x02

/* 設定の確認用（読み戻しのみ）。lsm6dsv16x_reg.h の LSM6DSV16X_HAODR_CFG (0x62): HAODR_SEL[1:0]。
 * ODR=7680Hz の通常モードでは 0 のままでよい（ドライバの xl_data_rate_set も 0x10 未満なら書かない）。 */
#define REG_HAODR_CFG      0x62

/* ====================================================================== */
/* 調整できる定数                                                          */
/* ====================================================================== */

/** 1回のCS区間（SPIのひと続きの通信）で読むFIFOワード数。
 *  提案: 既定は1（1ワードごとにCSを上げ下げ）。ST公式ドライバが確認できるのは
 *  「TAGから7バイトを1回で読む」までで、「7バイト読んだ後に続けて読むとアドレスが
 *  0x78に戻る（折り返す）」ことはドライバからは確認できなかったため。
 *  折り返しがデータシート/実機で確認できたら 16 などに増やすと CS の回数が減る。
 *  間違っていればタグが GYRO/ACCEL 以外になり imu_read_fifo が IMU_ERR_SPI を返す。 */
#ifndef IMU_FIFO_WORDS_PER_CS
#define IMU_FIFO_WORDS_PER_CS 1
#endif
_Static_assert(IMU_FIFO_WORDS_PER_CS >= 1 && IMU_FIFO_WORDS_PER_CS <= 32, "IMU_FIFO_WORDS_PER_CS は 1..32");

/** 1回の imu_read_fifo で読む最大ワード数 = 最大組数×2（ジャイロ+加速度）。 */
#define FIFO_READ_WORDS_MAX (IMU_BATCH_MAX_SAMPLES * 2)

/** SW_RESET 完了待ちの最大回数（1msごとに確認）。ドライバの sw_reset は3回だが余裕を見て10回（提案）。 */
#define RESET_POLL_MAX 10

/** 電源投入からSPIで話せるまでの待ち [ms]。ST公式サンプルの BOOT_TIME(10ms) に合わせた。 */
#define BOOT_TIME_MS 10

/* ====================================================================== */
/* 静的変数（固定長のみ。malloc しない）                                    */
/* ====================================================================== */

static imu_config_t s_cfg;
static bool         s_spi_ready = false;
static uint32_t     s_spi_actual_baud_hz = 0;

/* INT1 の記録。ISR が書き、メインループが読む。2つのフィールドを一緒に読むので、
 * 読む側は割り込みを止めてから写す。 */
static volatile uint32_t s_int1_count = 0;
static volatile uint32_t s_int1_timestamp_us = 0;
static uint32_t s_int1_last_seen = 0;   /* imu_wait_int1 が最後に受け取った count */

/* 感度（ドライバの換算関数 lsm6dsv16x_from_fsXXX_to_mdps / _to_mg の係数）。init で選ぶ。 */
static float s_gyro_rps_per_lsb = 0.0f;
static float s_accel_g_per_lsb  = 0.0f;
static uint8_t s_ctrl6_value = 0;
static uint8_t s_ctrl8_value = 0;

/* ジャイロと加速度が別々のワードで来るので、片方だけ届いた状態を呼び出しをまたいで持つ */
static bool     s_have_gyro = false, s_have_accel = false;
static vec3f_t  s_pend_gyro_rps, s_pend_accel_g;

/* SPI用の作業バッファ: 先頭1バイトがアドレス。最大 = FIFO 1回のCS区間 */
#define SPI_BUF_BYTES (1 + FIFO_WORD_BYTES * IMU_FIFO_WORDS_PER_CS)
static uint8_t s_tx[SPI_BUF_BYTES];
static uint8_t s_rx[SPI_BUF_BYTES];

/* ====================================================================== */
/* SPI の最小関数                                                          */
/* ====================================================================== */

static inline void cs_low(void)  { gpio_put(BOARD_IMU_CS_GPIO, 0); }
static inline void cs_high(void) { gpio_put(BOARD_IMU_CS_GPIO, 1); }

/** addr から len バイトを1回のCS区間で読む（len+1 <= SPI_BUF_BYTES）。 */
static imu_status_t reg_read(uint8_t addr, uint8_t *dst, size_t len)
{
    if (len + 1 > SPI_BUF_BYTES) {
        return IMU_ERR_SPI;
    }
    s_tx[0] = (uint8_t)(addr | 0x80u);          /* bit7=1 で読み出し */
    memset(&s_tx[1], 0, len);                   /* 読み出し中に送るダミー */
    cs_low();
    int n = spi_write_read_blocking(IMU_SPI, s_tx, s_rx, len + 1);
    cs_high();
    if (n != (int)(len + 1)) {
        return IMU_ERR_SPI;
    }
    memcpy(dst, &s_rx[1], len);
    return IMU_OK;
}

static imu_status_t reg_read1(uint8_t addr, uint8_t *dst)
{
    return reg_read(addr, dst, 1);
}

static imu_status_t reg_write1(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7Fu), val };   /* bit7=0 で書き込み */
    cs_low();
    int n = spi_write_blocking(IMU_SPI, tx, 2);
    cs_high();
    return (n == 2) ? IMU_OK : IMU_ERR_SPI;
}

/* ====================================================================== */
/* フルスケールと感度（ドライバの換算関数の係数）                            */
/* ====================================================================== */

/* 係数は lsm6dsv16x_reg.c の lsm6dsv16x_from_fs125_to_mdps ... from_fs4000_to_mdps。単位 mdps/LSB。 */
static bool select_gyro_fs(uint16_t dps, uint8_t *code, float *mdps_per_lsb)
{
    switch (dps) {
    case 125:  *code = 0x0; *mdps_per_lsb = 4.375f; return true;
    case 250:  *code = 0x1; *mdps_per_lsb = 8.75f;  return true;
    case 500:  *code = 0x2; *mdps_per_lsb = 17.5f;  return true;
    case 1000: *code = 0x3; *mdps_per_lsb = 35.0f;  return true;
    case 2000: *code = 0x4; *mdps_per_lsb = 70.0f;  return true;
    case 4000: *code = 0xC; *mdps_per_lsb = 140.0f; return true;
    default:   return false;
    }
}

/* 係数は lsm6dsv16x_reg.c の lsm6dsv16x_from_fs2_to_mg ... from_fs16_to_mg。単位 mg/LSB。 */
static bool select_accel_fs(uint8_t g, uint8_t *code, float *mg_per_lsb)
{
    switch (g) {
    case 2:  *code = 0x0; *mg_per_lsb = 0.061f; return true;
    case 4:  *code = 0x1; *mg_per_lsb = 0.122f; return true;
    case 8:  *code = 0x2; *mg_per_lsb = 0.244f; return true;
    case 16: *code = 0x3; *mg_per_lsb = 0.488f; return true;
    default: return false;
    }
}

/* ====================================================================== */
/* 設定の読み戻し                                                          */
/* ====================================================================== */

uint8_t imu_config_readback(imu_reg_readback_t *out, uint8_t max_entries)
{
    if (!s_spi_ready || out == NULL) {
        return 0;
    }
    const imu_reg_readback_t table[] = {
        { "WHO_AM_I",   REG_WHO_AM_I,   0, WHO_AM_I_VALUE, 0xFF },
        { "CTRL1(XL)",  REG_CTRL1,      0, (uint8_t)((OP_MODE_HIGH_PERF << 4) | ODR_7680HZ), 0xFF },
        { "CTRL2(G)",   REG_CTRL2,      0, (uint8_t)((OP_MODE_HIGH_PERF << 4) | ODR_7680HZ), 0xFF },
        { "CTRL3",      REG_CTRL3,      0, (uint8_t)(CTRL3_IF_INC | CTRL3_BDU), 0xFF },
        { "CTRL6(FS_G)", REG_CTRL6,     0, s_ctrl6_value, 0xFF },
        { "CTRL8(FS_XL)", REG_CTRL8,    0, s_ctrl8_value, 0xFF },
        { "FIFO_CTRL1(WTM)", REG_FIFO_CTRL1, 0, (uint8_t)s_cfg.fifo_watermark_words, 0xFF },
        { "FIFO_CTRL2", REG_FIFO_CTRL2, 0, 0x00, 0xFF },
        { "FIFO_CTRL3(BDR)", REG_FIFO_CTRL3, 0, (uint8_t)((BDR_7680HZ << 4) | BDR_7680HZ), 0xFF },
        { "FIFO_CTRL4(MODE)", REG_FIFO_CTRL4, 0, FIFO_MODE_STREAM, 0xFF },
        { "INT1_CTRL",  REG_INT1_CTRL,  0, INT1_CTRL_FIFO_TH, 0xFF },
        { "HAODR_CFG",  REG_HAODR_CFG,  0, 0x00, 0x03 },
    };
    const uint8_t n_table = (uint8_t)(sizeof(table) / sizeof(table[0]));
    _Static_assert(sizeof(table) / sizeof(table[0]) <= IMU_READBACK_MAX, "IMU_READBACK_MAX が足りない");

    uint8_t n = 0;
    for (uint8_t i = 0; i < n_table && n < max_entries; i++) {
        out[n] = table[i];
        if (reg_read1(table[i].addr, &out[n].value) != IMU_OK) {
            out[n].value = 0xEE;   /* 読めなかった印（期待値と必ず食い違うようにはしない: 表示で気づける） */
        }
        n++;
    }
    return n;
}

static imu_status_t verify_config(void)
{
    imu_reg_readback_t rb[IMU_READBACK_MAX];
    uint8_t n = imu_config_readback(rb, IMU_READBACK_MAX);
    if (n == 0) {
        return IMU_ERR_SPI;
    }
    for (uint8_t i = 0; i < n; i++) {
        if (((rb[i].value ^ rb[i].expected) & rb[i].mask) != 0) {
            return IMU_ERR_CONFIG;
        }
    }
    return IMU_OK;
}

/* ====================================================================== */
/* INT1 割り込み                                                           */
/* ====================================================================== */

void imu_on_int1_edge(void)
{
    /* ここでは SPI をしない。時刻と回数を残すだけ（読み出しは約160µsかかり、その間ほかの割り込みを止めてしまう） */
    s_int1_timestamp_us = time_us_32();
    s_int1_count = s_int1_count + 1u;
}

/* SDK のコールバック形式。登録したコア（コア0）で呼ばれる。
 * SDK はコアごとにコールバックを1つしか持てないので、将来ほかのGPIO割り込みを使うときは
 * ここで gpio 番号を見て振り分ける。 */
static void gpio_irq_cb(uint gpio, uint32_t events)
{
    if (gpio == BOARD_IMU_INT1_GPIO && (events & GPIO_IRQ_EDGE_RISE)) {
        imu_on_int1_edge();
    }
}

bool imu_int1_pin_is_high(void)
{
    return gpio_get(BOARD_IMU_INT1_GPIO);
}

bool imu_wait_int1(uint32_t timeout_us, imu_int1_event_t *ev)
{
    const uint32_t t0 = time_us_32();
    for (;;) {
        /* count は32bit書き込み1回なので、変化の有無だけなら割り込みを止めずに見てよい */
        if (s_int1_count != s_int1_last_seen) {
            /* timestamp と count を同じ時点のものとして写すため、割り込みを止める */
            uint32_t irq = save_and_disable_interrupts();
            uint32_t c = s_int1_count;
            uint32_t t = s_int1_timestamp_us;
            restore_interrupts(irq);
            s_int1_last_seen = c;
            if (ev != NULL) {
                ev->count = c;
                ev->timestamp_us = t;
            }
            return true;
        }
        if ((uint32_t)(time_us_32() - t0) >= timeout_us) {
            return false;
        }
    }
}

/* ====================================================================== */
/* 初期化                                                                  */
/* ====================================================================== */

imu_config_t imu_config_default(void)
{
    imu_config_t c;
    c.spi_baud_request_hz  = BOARD_IMU_SPI_BAUD_REQUEST_HZ;
    c.fifo_watermark_words = 16;      /* 8組（architecture.md #4 で決定） */
    c.odr_hz_measured      = 7519.0f; /* 設計メモ2章の実測値 */
    c.gyro_fs_dps          = 2000;    /* 提案: 機体の回転が速くても飽和しにくい値。仮 */
    c.accel_fs_g           = 16;      /* 提案: 衝撃（着地検出）も測れる値。仮 */
    return c;
}

uint32_t imu_spi_actual_baud_hz(void)
{
    return s_spi_actual_baud_hz;
}

imu_status_t imu_whoami_check(void)
{
    uint8_t id = 0;
    imu_status_t st = reg_read1(REG_WHO_AM_I, &id);
    if (st != IMU_OK) {
        return st;
    }
    return (id == WHO_AM_I_VALUE) ? IMU_OK : IMU_ERR_WHOAMI;
}

imu_status_t imu_fifo_flush(void)
{
    /* Bypass にするとFIFOが空になり、Stream に戻すと再び溜まり始める */
    imu_status_t st = reg_write1(REG_FIFO_CTRL4, FIFO_MODE_BYPASS);
    if (st != IMU_OK) {
        return st;
    }
    s_have_gyro = false;
    s_have_accel = false;
    return reg_write1(REG_FIFO_CTRL4, FIFO_MODE_STREAM);
}

imu_status_t imu_init(const imu_config_t *cfg)
{
    s_cfg = (cfg != NULL) ? *cfg : imu_config_default();

    /* 設定値の検査（SPIを触る前に） */
    float gyro_mdps = 0.0f, accel_mg = 0.0f;
    if (!select_gyro_fs(s_cfg.gyro_fs_dps, &s_ctrl6_value, &gyro_mdps) ||
        !select_accel_fs(s_cfg.accel_fs_g, &s_ctrl8_value, &accel_mg) ||
        s_cfg.fifo_watermark_words < 1 || s_cfg.fifo_watermark_words > 255) {
        return IMU_ERR_CONFIG;
    }
    /* rad/s = LSB × mdps/LSB × 1e-3 × π/180 */
    s_gyro_rps_per_lsb = gyro_mdps * 1.0e-3f * 0.017453292519943295f;
    s_accel_g_per_lsb  = accel_mg * 1.0e-3f;

    /* ---- SPI0 とピン ---- */
    /* 実際のクロックはペリフェラルクロックの分周で決まる（8MHz指定 → 150MHz÷20 = 7.5MHz）ので戻り値を保存 */
    s_spi_actual_baud_hz = spi_init(IMU_SPI, s_cfg.spi_baud_request_hz);
    spi_set_format(IMU_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);   /* モード0 */
    gpio_set_function(BOARD_IMU_MOSI_GPIO, GPIO_FUNC_SPI);
    gpio_set_function(BOARD_IMU_MISO_GPIO, GPIO_FUNC_SPI);
    gpio_set_function(BOARD_IMU_SCK_GPIO,  GPIO_FUNC_SPI);
    /* CS は SPIペリフェラルに任せず GPIO で操作する（複数バイトを1区間にまとめるため） */
    gpio_init(BOARD_IMU_CS_GPIO);
    gpio_set_dir(BOARD_IMU_CS_GPIO, GPIO_OUT);
    cs_high();
    s_spi_ready = true;

    sleep_ms(BOOT_TIME_MS);

    /* ---- WHO_AM_I ---- */
    imu_status_t st = imu_whoami_check();
    if (st != IMU_OK) {
        return st;
    }

    /* ---- ソフトリセット ---- */
    /* ドライバ lsm6dsv16x_sw_reset の手順: ①加速度・ジャイロをパワーダウン(ODR=0) ②CTRL3.SW_RESET=1
     * ③SW_RESET が0に戻るまで待つ。リセット後は全レジスタが既定値になる。 */
    if ((st = reg_write1(REG_CTRL1, 0x00)) != IMU_OK) return st;
    if ((st = reg_write1(REG_CTRL2, 0x00)) != IMU_OK) return st;
    if ((st = reg_write1(REG_CTRL3, CTRL3_SW_RESET)) != IMU_OK) return st;
    bool reset_done = false;
    for (int i = 0; i < RESET_POLL_MAX; i++) {
        sleep_ms(1);
        uint8_t c3 = 0;
        if ((st = reg_read1(REG_CTRL3, &c3)) != IMU_OK) return st;
        if ((c3 & CTRL3_SW_RESET) == 0) {
            reset_done = true;
            break;
        }
    }
    if (!reset_done) {
        return IMU_ERR_CONFIG;
    }

    /* ---- 設定（ODRを入れる前に全部済ませる）---- */
    /* リセット直後なので、書かないビットは既定値（0）のままになる。バイト丸ごと書く。 */
    if ((st = reg_write1(REG_CTRL3, (uint8_t)(CTRL3_IF_INC | CTRL3_BDU))) != IMU_OK) return st;
    if ((st = reg_write1(REG_CTRL6, s_ctrl6_value)) != IMU_OK) return st;
    if ((st = reg_write1(REG_CTRL8, s_ctrl8_value)) != IMU_OK) return st;

    if ((st = reg_write1(REG_FIFO_CTRL1, (uint8_t)s_cfg.fifo_watermark_words)) != IMU_OK) return st;
    if ((st = reg_write1(REG_FIFO_CTRL2, 0x00)) != IMU_OK) return st;
    if ((st = reg_write1(REG_FIFO_CTRL3, (uint8_t)((BDR_7680HZ << 4) | BDR_7680HZ))) != IMU_OK) return st;
    if ((st = reg_write1(REG_FIFO_CTRL4, FIFO_MODE_STREAM)) != IMU_OK) return st;
    if ((st = reg_write1(REG_INT1_CTRL, INT1_CTRL_FIFO_TH)) != IMU_OK) return st;

    /* ---- INT1 (GP26): 入力・立ち上がりエッジ割り込み。コア0で登録する ---- */
    /* プルダウン: 配線が外れていてもピンが浮かず、「INT1が来ない」がはっきり出る。
     * IMU側は既定でプッシュプル・アクティブHigh（IF_CFG の既定）なのでプルダウンと喧嘩しない。 */
    gpio_init(BOARD_IMU_INT1_GPIO);
    gpio_set_dir(BOARD_IMU_INT1_GPIO, GPIO_IN);
    gpio_pull_down(BOARD_IMU_INT1_GPIO);
    s_int1_last_seen = s_int1_count;
    /* ODRを入れる前に登録する: 入れた後だと最初の立ち上がりを逃し、ピンがHighのまま止まるおそれがあるため */
    gpio_set_irq_enabled_with_callback(BOARD_IMU_INT1_GPIO, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_cb);

    /* ---- ODR を入れる（ここからFIFOに溜まり始める）---- */
    if ((st = reg_write1(REG_CTRL1, (uint8_t)((OP_MODE_HIGH_PERF << 4) | ODR_7680HZ))) != IMU_OK) return st;
    if ((st = reg_write1(REG_CTRL2, (uint8_t)((OP_MODE_HIGH_PERF << 4) | ODR_7680HZ))) != IMU_OK) return st;

    /* ---- 読み戻して確認 ---- */
    return verify_config();
}

float imu_sample_period_s(void)
{
    return (s_cfg.odr_hz_measured > 0.0f) ? (1.0f / s_cfg.odr_hz_measured) : (1.0f / 7680.0f);
}

/* ====================================================================== */
/* FIFO 読み出し                                                           */
/* ====================================================================== */

static inline int16_t le16(const uint8_t *p)
{
    return (int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/** 1ワード(TAG+6バイト)を物理単位に直して、組になるものを out へ。不正なタグなら false。 */
static bool consume_word(const uint8_t *w, imu_batch_t *out)
{
    const uint8_t tag = (uint8_t)(w[0] >> 3);   /* TAG_SENSOR[7:3] */
    const float x = (float)le16(&w[1]);
    const float y = (float)le16(&w[3]);
    const float z = (float)le16(&w[5]);

    if (tag == TAG_GYRO) {
        s_pend_gyro_rps.x = x * s_gyro_rps_per_lsb;
        s_pend_gyro_rps.y = y * s_gyro_rps_per_lsb;
        s_pend_gyro_rps.z = z * s_gyro_rps_per_lsb;
        s_have_gyro = true;
    } else if (tag == TAG_ACCEL) {
        s_pend_accel_g.x = x * s_accel_g_per_lsb;
        s_pend_accel_g.y = y * s_accel_g_per_lsb;
        s_pend_accel_g.z = z * s_accel_g_per_lsb;
        s_have_accel = true;
    } else {
        return false;
    }

    /* 両方そろったら1組にする（到着順に依存しない。片方だけ余った分は次回に持ち越す）。
     * 同じ種類が続けて来たら新しい方で上書きされる（取りこぼしがあれば組がずれうる）。 */
    if (s_have_gyro && s_have_accel && out->count < IMU_BATCH_MAX_SAMPLES) {
        out->samples[out->count].gyro_rps = s_pend_gyro_rps;
        out->samples[out->count].accel_g  = s_pend_accel_g;
        out->count++;
        s_have_gyro = false;
        s_have_accel = false;
    }
    return true;
}

imu_status_t imu_read_fifo(imu_batch_t *out)
{
    out->count = 0;
    out->words_read = 0;
    out->fifo_overrun = false;
    out->truncated = false;

    /* 未読ワード数 DIFF_FIFO (9bit) と OVR を、FIFO_STATUS1/2 の2バイトで取る */
    uint8_t st12[2];
    imu_status_t st = reg_read(REG_FIFO_STATUS1, st12, 2);
    if (st != IMU_OK) {
        return st;
    }
    uint16_t diff = (uint16_t)(((st12[1] & FIFO_STATUS2_DIFF8) ? 256u : 0u) | st12[0]);
    /* あふれは FIFO_OVR_IA（ドライバの fifo_status_get と同じ）に加え、読むまで保持される
     * FIFO_OVR_LATCHED も見る（提案。取りこぼし防止。ラッチの解除条件は未確認なので、
     * もし増え続けるなら外す）。 */
    out->fifo_overrun = (st12[1] & (FIFO_STATUS2_OVR_IA | FIFO_STATUS2_OVR_LATCH)) != 0;

    uint16_t to_read = diff;
    if (to_read > FIFO_READ_WORDS_MAX) {
        to_read = FIFO_READ_WORDS_MAX;
        out->truncated = true;   /* 残りは次回 */
    }

    bool bad_tag = false;
    uint16_t done = 0;
    while (done < to_read) {
        uint16_t n = to_read - done;
        if (n > IMU_FIFO_WORDS_PER_CS) {
            n = IMU_FIFO_WORDS_PER_CS;
        }
        uint8_t buf[FIFO_WORD_BYTES * IMU_FIFO_WORDS_PER_CS];
        st = reg_read(REG_FIFO_DATA_OUT_TAG, buf, (size_t)n * FIFO_WORD_BYTES);
        if (st != IMU_OK) {
            out->words_read = done;
            return st;
        }
        for (uint16_t i = 0; i < n; i++) {
            if (!consume_word(&buf[i * FIFO_WORD_BYTES], out)) {
                bad_tag = true;
            }
        }
        done = (uint16_t)(done + n);
    }
    out->words_read = done;

    /* 想定外のタグ（空・タイムスタンプなど）が来たら、FIFOの設定か読み方がおかしい */
    return bad_tag ? IMU_ERR_SPI : IMU_OK;
}
