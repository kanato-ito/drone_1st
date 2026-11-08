/**
 * @file controller.c
 * @brief 制御器の共通部分: 入力の作成と14要素への詰め込み（controller.h）。
 */
#include "controller.h"

void controller_input_make(const attitude_state_t *att, const euler_t *target, float target_thrust_g,
                           const controller_output_t *prev, controller_input_t *in)
{
    in->attitude = att->angle;
    /* ヨーは「目標とのずれ」を ±π に折り返して渡し、目標ヨーは0にする（rev22） */
    in->attitude.yaw_rad = attitude_wrap_pi(att->angle.yaw_rad - target->yaw_rad);
    in->rate_rps = att->rate_rps;
    in->target_attitude.roll_rad = target->roll_rad;
    in->target_attitude.pitch_rad = target->pitch_rad;
    in->target_attitude.yaw_rad = 0.0f;
    in->target_thrust_g = target_thrust_g;
    in->prev_output = *prev;
}

void controller_input_pack(const controller_input_t *in, float out[FC_CONTROLLER_INPUT_DIM])
{
    out[0] = in->attitude.roll_rad;
    out[1] = in->attitude.pitch_rad;
    out[2] = in->attitude.yaw_rad;
    out[3] = in->rate_rps.x;
    out[4] = in->rate_rps.y;
    out[5] = in->rate_rps.z;
    out[6] = in->target_attitude.roll_rad;
    out[7] = in->target_attitude.pitch_rad;
    out[8] = in->target_attitude.yaw_rad;
    out[9] = in->target_thrust_g;
    out[10] = in->prev_output.thrust_g;
    out[11] = in->prev_output.torque_g[0];
    out[12] = in->prev_output.torque_g[1];
    out[13] = in->prev_output.torque_g[2];
}
