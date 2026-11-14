/**
 * @file flight_state.c
 * @brief 状態6つの遷移（flight_state.h）。入力の構造体から次の状態を決める純粋関数。SDK非依存。
 *
 * 遷移表（優先順に上から評価する。docs/step3b_state_safety.md に同じ表がある）:
 *   BOOT          : WDT再起動 → ESTOP(WDT) / IMU初期化失敗 → ESTOP(IMU_INIT_FAIL) / 非常停止旗 → ESTOP(OPERATOR) / それ以外 → WAIT_COMM
 *   WAIT_COMM     : 非常停止旗 / IMU途絶 → ESTOP / 通信接続 → STANDBY
 *   STANDBY       : 非常停止旗 / IMU途絶 → ESTOP / 通信途絶 → WAIT_COMM(提案)
 *                   / アーム旗の0→1 かつ 水平・スロットル0 かつ ディスアーム要求なし → FLIGHT
 *   FLIGHT        : 非常停止旗 / IMU途絶 / 傾き過大 → ESTOP / ディスアーム → STANDBY / 通信途絶・電池低下 → DESCENT
 *   DESCENT       : 非常停止旗 / IMU途絶(提案) / 傾き過大(提案) → ESTOP / 着地 → ESTOP / 時間切れ → ESTOP
 *   ESTOP         : 何があっても抜けない
 */
#include <math.h>
#include "flight_state.h"

void flight_fsm_init(flight_fsm_t *f, uint64_t now_us)
{
    f->state = FS_BOOT;
    f->estop_reason = ESTOP_NONE;
    f->descent_reason = DESCENT_NONE;
    f->state_enter_us = now_us;
}

static bool go(flight_fsm_t *f, flight_state_t s, uint64_t now_us)
{
    f->state = s;
    f->state_enter_us = now_us;
    return true;
}

static bool go_estop(flight_fsm_t *f, estop_reason_t r, uint64_t now_us)
{
    f->estop_reason = r;
    return go(f, FS_EMERGENCY_STOP, now_us);
}

static bool go_descent(flight_fsm_t *f, descent_reason_t r, uint64_t now_us)
{
    f->descent_reason = r;
    return go(f, FS_EMERGENCY_DESCENT, now_us);
}

bool flight_fsm_step(flight_fsm_t *f, const flight_fsm_inputs_t *in, float tilt_estop_rad, uint64_t now_us)
{
    /* 非常停止からは抜けない（電源入れ直しのみ） */
    if (f->state == FS_EMERGENCY_STOP) return false;

    if (f->state == FS_BOOT) {
        /* 起動の原因（WDT・IMU）のほうが操作より根本なので先に見る */
        if (in->wdt_reboot) return go_estop(f, ESTOP_WDT_REBOOT, now_us);
        if (!in->boot_imu_ok) return go_estop(f, ESTOP_IMU_INIT_FAIL, now_us);
        if (in->estop_request) return go_estop(f, ESTOP_OPERATOR, now_us);
        return go(f, FS_WAIT_COMM, now_us);
    }

    /* 操作者の非常停止は、どの状態からでも */
    if (in->estop_request) return go_estop(f, ESTOP_OPERATOR, now_us);

    /* 傾きは NaN も異常とみなす（!(x <= 限度) の形） */
    const bool tilt_bad = !(in->tilt_rad <= tilt_estop_rad);

    switch (f->state) {
    case FS_WAIT_COMM:
        /* 提案: 停止状態でもIMUが無応答なら非常停止（飛べない機体を待機させない） */
        if (in->imu_timeout) return go_estop(f, ESTOP_IMU_TIMEOUT, now_us);
        if (in->comm_connected && !in->comm_lost) return go(f, FS_STANDBY, now_us);
        return false;

    case FS_STANDBY:
        if (in->imu_timeout) return go_estop(f, ESTOP_IMU_TIMEOUT, now_us);
        /* 提案(#20): 待機中の通信途絶は通信待機へ戻す */
        if (in->comm_lost) return go(f, FS_WAIT_COMM, now_us);
        /* アームは「旗の0→1」の周期だけ。ディスアーム要求と同時なら見送る（提案） */
        if (in->arm_rising_edge && in->level_and_zero_throttle && !in->disarm_request) {
            return go(f, FS_FLIGHT, now_us);
        }
        return false;

    case FS_FLIGHT:
        if (in->imu_timeout) return go_estop(f, ESTOP_IMU_TIMEOUT, now_us);
        if (tilt_bad) return go_estop(f, ESTOP_TILT_EXCEEDED, now_us);
        if (in->disarm_request) return go(f, FS_STANDBY, now_us);
        if (in->comm_lost) return go_descent(f, DESCENT_COMM_LOST, now_us);
        if (in->battery_low) return go_descent(f, DESCENT_BATTERY_LOW, now_us);
        return false;

    case FS_EMERGENCY_DESCENT:
        /* 提案: 降下中もIMU途絶・傾き過大は非常停止（姿勢が分からない・戻せない機体を回し続けない）。
         * ディスアーム旗は降下中は無視する（機体が自分で降りる。止めたいときは非常停止旗） */
        if (in->imu_timeout) return go_estop(f, ESTOP_IMU_TIMEOUT, now_us);
        if (tilt_bad) return go_estop(f, ESTOP_TILT_EXCEEDED, now_us);
        if (in->landed) return go_estop(f, ESTOP_DESCENT_LANDED, now_us);
        if (in->descent_timeout) return go_estop(f, ESTOP_DESCENT_TIMEOUT, now_us);
        return false;

    default:
        /* 未知の状態値（メモリ破損など）は安全側に倒す */
        return go_estop(f, ESTOP_INTERNAL_ERROR, now_us);
    }
}

bool flight_fsm_motors_allowed(flight_state_t s)
{
    return s == FS_FLIGHT || s == FS_EMERGENCY_DESCENT;
}

const char *flight_state_name(flight_state_t s)
{
    switch (s) {
    case FS_BOOT:              return "BOOT";
    case FS_WAIT_COMM:         return "WAIT_COMM";
    case FS_STANDBY:           return "STANDBY";
    case FS_FLIGHT:            return "FLIGHT";
    case FS_EMERGENCY_DESCENT: return "DESCENT";
    case FS_EMERGENCY_STOP:    return "ESTOP";
    default:                   return "?";
    }
}

const char *estop_reason_name(estop_reason_t r)
{
    switch (r) {
    case ESTOP_NONE:            return "none";
    case ESTOP_IMU_INIT_FAIL:   return "imu_init_fail";
    case ESTOP_WDT_REBOOT:      return "wdt_reboot";
    case ESTOP_IMU_TIMEOUT:     return "imu_timeout";
    case ESTOP_TILT_EXCEEDED:   return "tilt_exceeded";
    case ESTOP_OPERATOR:        return "operator";
    case ESTOP_DESCENT_LANDED:  return "landed";
    case ESTOP_DESCENT_TIMEOUT: return "descent_timeout";
    case ESTOP_INTERNAL_ERROR:  return "internal_error";
    default:                    return "?";
    }
}

const char *descent_reason_name(descent_reason_t r)
{
    switch (r) {
    case DESCENT_NONE:         return "none";
    case DESCENT_COMM_LOST:    return "comm_lost";
    case DESCENT_BATTERY_LOW:  return "battery_low";
    default:                   return "?";
    }
}
