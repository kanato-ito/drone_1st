/**
 * @file ipc_flags.h
 * @brief 旗。コア1が要求を出し、コア0が処理済みを返す（設計メモ4章）。
 *
 * 差し替え境界（ポート）ではない: 実装（ステップ④）は src/ の純粋C11（<stdatomic.h>）で書く。
 * シミュレータは本物をそのままリンクし、コア1の役を自分で演じる。
 *
 * 「1つの変数に書く人は1人」（spsc_ring の head/tail と同じ考え方）。語は3つ:
 *   estop      ... コア1だけが書く。0→1にするだけ（下ろさない）。
 *   disarm_req ... コア1だけが書く。ディスアーム要求のたびに +1（カウンタ）。
 *   disarm_ack ... コア0だけが書く。処理時に disarm_req の値を写す。
 * 未処理のディスアーム = (disarm_req != disarm_ack)。コア0が読む前に何回来ても
 * 「1回分の未処理」として扱い、取りこぼさない（ディスアームは冪等）。
 * 非常停止は抜けない（メモ5章）ので、立てるだけで足りる。
 * 読み-変更-書きを2つのコアで奪い合わないため、RP2350のコア間排他モニタの有無に
 * 依存しない。語は32ビット単位の _Atomic load/store のみで使う（fetch_or等は使わない）。
 * disarm_req の +1 は書く人が1人なので、load して store するだけでよい。
 */
#ifndef IPC_FLAGS_H
#define IPC_FLAGS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

typedef struct {
    _Atomic uint32_t estop;       /**< 書く人: コア1だけ */
    _Atomic uint32_t disarm_req;  /**< 書く人: コア1だけ */
    _Atomic uint32_t disarm_ack;  /**< 書く人: コア0だけ */
} ipc_flags_t;

/** 全部0にする。コア0、コア1起動前に1回。 */
void ipc_flags_init(ipc_flags_t *f);

/** 非常停止を要求（estop を1にするだけ）。コア1だけ。非常停止旗付きパケット受信ごと。 */
void ipc_flags_request_estop(ipc_flags_t *f);

/** ディスアームを要求（disarm_req を +1）。コア1だけ。ディスアーム旗付きパケット受信ごと。 */
void ipc_flags_request_disarm(ipc_flags_t *f);

/** 非常停止が要求されているか。コア0、1周期に1回。 */
bool ipc_flags_estop_requested(const ipc_flags_t *f);

/** 未処理のディスアーム要求があるか（disarm_req != disarm_ack）。コア0、1周期に1回。 */
bool ipc_flags_disarm_pending(const ipc_flags_t *f);

/** ディスアームを処理済みにする（disarm_ack に disarm_req を写す）。コア0だけ。処理したとき。 */
void ipc_flags_disarm_acknowledge(ipc_flags_t *f);

#endif /* IPC_FLAGS_H */
