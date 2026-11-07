/**
 * @file controller_policy.h
 * @brief 強化学習の方策（全結合ネットワーク）。コア0、ループの中で約940Hz。
 *
 * 見積もり: 14→64→64→4 で5248積和、約0.18ms、重み・バイアス5380個（約21.5KB）。入力14は2026-11-07決定。
 * ★メモは入力を「14個」としつつ式は「15→」と書いている（architecture.md #2）。
 *   ここでは14（FC_CONTROLLER_INPUT_DIM）を採用し、次元はマクロで追従する。
 * 重みは固定長のRAM配列を2面持つ（受信用と使用中）。飛行中のmallocなし。
 * 重みの更新は飛行指示待機のときだけ（設計メモ4,6章）。
 */
#ifndef CONTROLLER_POLICY_H
#define CONTROLLER_POLICY_H

#include <stdint.h>
#include <stdbool.h>
#include "controller.h"
#include "params.h"

#define POLICY_HIDDEN 64

/** float個数: W1+b1+W2+b2+W3+b3（この順、行優先。メモ未記載、提案）。 */
#define POLICY_WEIGHT_FLOATS \
    (FC_CONTROLLER_INPUT_DIM * POLICY_HIDDEN + POLICY_HIDDEN + \
     POLICY_HIDDEN * POLICY_HIDDEN + POLICY_HIDDEN + \
     POLICY_HIDDEN * FC_CONTROLLER_OUTPUT_DIM + FC_CONTROLLER_OUTPUT_DIM)

typedef enum {
    POLICY_ACT_RELU = 0,  /**< 活性化関数はメモ未記載。学習側と合わせて決める */
    POLICY_ACT_TANH = 1
} policy_activation_t;

typedef struct {
    float weights[2][POLICY_WEIGHT_FLOATS]; /**< [0]/[1] を active_idx で切替 */
    uint8_t active_idx;
    bool    loaded;                 /**< 有効な重みが確定済み */
    uint32_t staged_floats;         /**< 受信済み個数 */
    policy_activation_t activation;
    const policy_norm_params_t *norm;
} policy_controller_t;

/** 初期化（重み未ロード）。コア0、起動時に1回。 */
void policy_init(policy_controller_t *p, const policy_norm_params_t *norm);

/** 共通インターフェースのハンドルを返す。コア0、起動時に1回。未ロードの step は false。 */
controller_t policy_handle(policy_controller_t *p);

/**
 * 重みの断片を受信用の面に書く。
 * 呼ぶコア: コア0（コマンドキューの処理中）。頻度: 重み転送中のみ、断片ごと。飛行指示待機のみ。
 */
bool policy_stage_chunk(policy_controller_t *p, uint32_t offset_floats, const float *data, uint16_t count);

/**
 * 受信完了を確定し、チェックサム(CRC32)が合えば使用面を切り替える。
 * コア0、重み転送の最後に1回、飛行指示待機のみ。失敗時は元の重みのまま。
 */
bool policy_commit(policy_controller_t *p, uint32_t crc32, policy_activation_t act);

#endif /* CONTROLLER_POLICY_H */
