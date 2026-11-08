/**
 * @file imu_mount.h
 * @brief IMU基板の軸 → 機体の軸の並べ替え（params の imu_mount.axis_map）。純粋C、SDK非依存。
 *
 * axis_map[i] = ±(1..3): 機体の軸 i（0=x,1=y,2=z）は、IMU基板の軸 |v| に符号 sign(v) を掛けたもの。
 * 書き方の例（実機の値ではない）: {-3,1,2} なら 機体x = -基板z、機体y = +基板x、機体z = +基板y。
 * ドライバ（実機もシミュレータの偽物も）は基板の軸のまま出す。並べ替えはここで1回だけ行う。
 * 偽物は並べ替えてはいけない（二重に変換される）。
 */
#ifndef IMU_MOUNT_H
#define IMU_MOUNT_H

#include <stdbool.h>
#include "params.h"
#include "imu_lsm6dsv16x.h"

/** 各要素の絶対値が 1..3 で、3つの軸が重ならないなら true。コア0、起動時とパラメータ変更時。 */
bool imu_mount_valid(const imu_mount_params_t *m);

/**
 * batch の全サンプルの gyro と accel を、その場で機体の軸へ並べ替える。
 * m が不正なら何もしない。コア0、1周期に1回（imu_read_fifo の直後）。
 */
void imu_mount_apply(const imu_mount_params_t *m, imu_batch_t *b);

#endif /* IMU_MOUNT_H */
