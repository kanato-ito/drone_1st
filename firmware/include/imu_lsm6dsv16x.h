/**
 * @file imu_lsm6dsv16x.h
 * @brief IMU（LSM6DSV16X, SPI0）ドライバ。INT1で起こされ、FIFOを全部読む。
 *
 * - すべてコア0で使う。SDK非依存（SPI/GPIOは .c の中に隠す）。
 * - INT1(GP26, 立ち上がり)の割り込み登録はコア0で行うこと。
 *   Pico SDKのGPIO割り込みはコアごとに有効化されるため、コア1でWi-Fiを
 *   初期化しても影響しない（設計メモ3章の分担を保つ）。
 * - レジスタ名は STの公式ドライバ(lsm6dsv16x-pid)の記憶に基づく。アドレス値と
 *   ビット位置はここには書かず、.c 実装時にデータシートで確認する:
 *     WHO_AM_I の期待値            ... TODO: DS確認
 *     FIFO_CTRL1 (FIFO_WTM)        ... TODO: DS確認（単位が「ワード」か要確認）
 *     FIFO_STATUS1/2 (DIFF_FIFO, FIFO_WTM_IA, FIFO_OVR_IA)  ... TODO: DS確認
 *     FIFO_DATA_OUT_TAG / FIFO_DATA_OUT_X_L ...（1ワード=タグ1+データ6バイト）TODO: DS確認
 *     タグ値（ジャイロ/加速度）    ... TODO: DS確認
 */
#ifndef IMU_LSM6DSV16X_H
#define IMU_LSM6DSV16X_H

#include <stdint.h>
#include <stdbool.h>
#include "fc_types.h"

/** 1周期で受け取る最大サンプル組数（提案値）。通常8組。遅れて溜まった分の上限。 */
#define IMU_BATCH_MAX_SAMPLES 32

/** 1組 = 同時刻のジャイロ+加速度（物理単位に変換済み）。 */
typedef struct {
    vec3f_t gyro_rps;  /**< 角速度 [rad/s]（機体座標系、軸変換・オフセット補正済みかは .c で決める） */
    vec3f_t accel_g;   /**< 加速度 [g] */
} imu_sample_t;

/** FIFOから1周期ぶん読み出した結果。固定長（mallocしない）。 */
typedef struct {
    uint8_t  count;            /**< samples[] の有効数 */
    uint16_t words_read;       /**< 読んだFIFOワード数（ログ用。通常16） */
    bool     fifo_overrun;     /**< FIFOあふれ（FIFO_OVR_IA）を検出した */
    bool     truncated;        /**< IMU_BATCH_MAX_SAMPLES に達し、FIFOに読み残しがある */
    imu_sample_t samples[IMU_BATCH_MAX_SAMPLES];
} imu_batch_t;

typedef enum {
    IMU_OK = 0,
    IMU_ERR_WHOAMI,   /**< WHO_AM_I が期待値と違う（配線・無応答） */
    IMU_ERR_SPI,      /**< SPI通信の失敗 */
    IMU_ERR_CONFIG    /**< 設定の書き込み・読み戻しが一致しない */
} imu_status_t;

/** IMU設定。imu_config_default() で既定値を得て、必要なら書き換える。 */
typedef struct {
    uint32_t spi_baud_request_hz;  /**< 8000000（実7.5MHz）。設計メモ7章 #6 */
    uint16_t fifo_watermark_words; /**< ウォーターマーク。単位が「ワード」なら16（8組）。architecture.md #4 */
    float    odr_hz_measured;      /**< 実測ODR [Hz]（設計メモ2章: 7519）。姿勢推定のdtに使う */
    uint16_t gyro_fs_dps;         /**< ジャイロのフルスケール [dps]。提案: 要検討（メモ未記載） */
    uint8_t  accel_fs_g;          /**< 加速度のフルスケール [g]。提案: 要検討（メモ未記載） */
} imu_config_t;

/**
 * INT1の発生記録。ISRが書き、メインループが読む（どちらもコア0）。
 * 実体（.cの静的変数）は volatile にすること。2つのフィールドを一緒に読むので、
 * メインループ側は読み出し時に割り込みを止めて ev に写す（または count を読み直して
 * 変化していれば再読み）。imu_wait_int1() の内部でこれを行う。
 */
typedef struct {
    uint32_t count;         /**< INT1立ち上がりの累計回数 */
    uint32_t timestamp_us;  /**< 最新の立ち上がり時刻 [µs]（下位32bit）。1周期の「開始時刻」 */
} imu_int1_event_t;

/** 既定のIMU設定を返す。コア0、起動時に1回。 */
imu_config_t imu_config_default(void);

/**
 * SPI0・CS・INT1を初期化し、IMUを設定する（WHO_AM_I確認、FIFO設定、ODR設定）。
 * 呼ぶコア: コア0。頻度: 起動時に1回。INT1の割り込み登録もここで行う。
 * 失敗時は IMU_ERR_*。呼び出し側は非常停止（理由: IMU初期化失敗）へ移す（設計メモ5章）。
 */
imu_status_t imu_init(const imu_config_t *cfg);

/**
 * WHO_AM_I を1回読んで期待値と比べる。
 * 呼ぶコア: コア0。頻度: ブレッドボード試験で100万回、通常は起動時に1回。
 * 設計メモ7章 #6（SPIクロック確定試験）で使う。
 */
imu_status_t imu_whoami_check(void);

/** 実際に設定されたSPIクロック [Hz]（spi_init()の戻り値）。ログ・試験用。コア0、随時。 */
uint32_t imu_spi_actual_baud_hz(void);

/** FIFOを空にする（古いデータを捨てる）。コア0、起動時とエラー復帰時のみ。 */
imu_status_t imu_fifo_flush(void);

/**
 * INT1立ち上がりの割り込みハンドラ本体。
 * 呼ぶコア: コア0（割り込みコンテキスト）。頻度: 約940Hz。
 * 中ではSPIをしない。時刻と回数を記録するだけ（理由は architecture.md 参照）。
 * SDKのコールバック(.c側)から呼ぶ。
 */
void imu_on_int1_edge(void);

/**
 * INT1が来るまで待つ。timeout_us 経過したら false を返す。
 * 呼ぶコア: コア0（制御ループの先頭）。頻度: 1周期に1回。
 * timeout_us には FC_INT1_TIMEOUT_US (約5ms) を渡す。
 * true のとき ev に最新のINT1記録を入れる。
 */
bool imu_wait_int1(uint32_t timeout_us, imu_int1_event_t *ev);

/**
 * INT1ピンの現在のレベル（Highなら true）。
 * 呼ぶコア: コア0。頻度: FIFO読み出し後に1回。
 * 立ち上がりエッジ割り込みは、読み終わってもピンがHighのままだと次のエッジが
 * 来ず周期が止まる。その取りこぼしを検出するために使う（architecture.md #10）。
 */
bool imu_int1_pin_is_high(void);

/**
 * FIFOにたまっている分を全部読み、サンプル組に変換して out に入れる。
 * 呼ぶコア: コア0。頻度: 1周期に1回（約940Hz）。通常16ワード（8組）。
 * 所要: 1ワード約10µs×16 ≒ 160µs（再見積もり、要実測）。ブロッキング。
 * ジャイロ/加速度のタグで組にする。失敗時は IMU_ERR_SPI（呼び出し側が連続失敗を数える）。
 */
imu_status_t imu_read_fifo(imu_batch_t *out);

/** サンプル間隔 [s] = 1 / odr_hz_measured。姿勢推定のdtに使う。コア0、随時。 */
float imu_sample_period_s(void);

#endif /* IMU_LSM6DSV16X_H */
