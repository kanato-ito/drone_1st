/**
 * @file imu_mount.c
 * @brief IMU基板の軸 → 機体の軸の並べ替え（imu_mount.h）。
 */
#include "imu_mount.h"

static int abs_i(int v) { return v < 0 ? -v : v; }

bool imu_mount_valid(const imu_mount_params_t *m)
{
    for (int i = 0; i < 3; i++) {
        int a = abs_i(m->axis_map[i]);
        if (a < 1 || a > 3) return false;
    }
    int a0 = abs_i(m->axis_map[0]), a1 = abs_i(m->axis_map[1]), a2 = abs_i(m->axis_map[2]);
    return a0 != a1 && a0 != a2 && a1 != a2;
}

static vec3f_t remap(const imu_mount_params_t *m, vec3f_t v)
{
    const float in[3] = { v.x, v.y, v.z };
    float out[3];
    for (int i = 0; i < 3; i++) {
        int a = m->axis_map[i];
        float s = in[abs_i(a) - 1];
        out[i] = a < 0 ? -s : s;
    }
    vec3f_t r = { out[0], out[1], out[2] };
    return r;
}

void imu_mount_apply(const imu_mount_params_t *m, imu_batch_t *b)
{
    if (!imu_mount_valid(m)) return;
    for (int k = 0; k < b->count && k < IMU_BATCH_MAX_SAMPLES; k++) {
        b->samples[k].gyro_rps = remap(m, b->samples[k].gyro_rps);
        b->samples[k].accel_g  = remap(m, b->samples[k].accel_g);
    }
}
