/**
 * @file control_loop.c
 * @brief コア0の制御ループ本体（control_loop.h）。純粋C11。SDKには触れず、ポートのヘッダ越しにだけ呼ぶ。
 *
 * 1周期の順序（docs/architecture.md「1周期の流れ」。ステップ④で入る部分は TODO(④)）:
 *   timing開始 → imu_read_fifo → imu_mount_apply → 姿勢 → 電池(数周期に1回) → mailbox / ipc_flags
 *   → （降下中なら着地判定）→ 状態機械 → 制御器の選択と実行（PIDは常に影として） → 見張り役
 *   → thrust_model + mixer → motor_pwm_set / motor_pwm_stop → timing終了 → ログ組み立て
 *   → TODO(④) ログをキューへ → fc_wdt_feed → INT1ピンの確認
 */
#include <math.h>
#include <string.h>
#include "control_loop.h"
#include "imu_mount.h"
#include "thrust_model.h"
#include "motor_pwm.h"

#define LOOP_BUDGET_US 1064u   /* 1周期（IMUの実周期）。超過回数の統計用 */

static float finite_or_zero(float v) { return isfinite(v) ? v : 0.0f; }

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static controller_output_t zero_output(void)
{
    controller_output_t o = { 0.0f, { 0.0f, 0.0f, 0.0f } };
    return o;
}

static bool policy_available(const control_loop_t *c)
{
    return c->policy_ctrl.ops != NULL;
}

/* ---------------------------------------------------------------- 初期化 */

void control_loop_init(control_loop_t *c, const params_t *params)
{
    /* 最初に必ずモーターを0にする（以降どの経路でもPWMは非常停止側に倒れる） */
    motor_pwm_init();
    motor_pwm_stop();

    memset(c, 0, sizeof(*c));
    c->params = params;
    c->motor_output_enabled = true;
    c->motor_duty_max = 1.0f;
    c->selected_controller = CONTROLLER_PID;
    c->active_controller = CONTROLLER_PID;

    const uint64_t now = fc_clock_now_us64();
    flight_fsm_init(&c->fsm, now);

    /* IMU。失敗は起動の遷移（BOOT→ESTOP）に渡す */
    imu_config_t cfg = imu_config_default();
    c->boot_imu_ok = (imu_init(&cfg) == IMU_OK);
    c->wdt_reboot = fc_wdt_caused_reboot();
    c->dt_sample = imu_sample_period_s();
    c->fifo_wtm_words = cfg.fifo_watermark_words;

    attitude_init(&c->att, &params->attitude);
    controller_pid_init(&c->pid, &params->pid);
    c->pid_ctrl = controller_pid_handle(&c->pid);
    /* 方策: 実装が未着手なので policy_init は呼ばない。policy_ctrl.ops==NULL = 使えない */
    guard_reset(&c->guard);
    descent_init(&c->descent);
    battery_init(&c->batt);
    battery_update(&c->batt, &params->battery, (uint32_t)(now / 1000u));   /* 最初の1回でローパスを初期化 */
    c->batt_v_used = c->batt.voltage_v;

    /* BOOT の遷移（WDT再起動・IMU無応答なら非常停止） */
    flight_fsm_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.boot_imu_ok = c->boot_imu_ok;
    in.wdt_reboot = c->wdt_reboot;
    flight_fsm_step(&c->fsm, &in, params->safety.tilt_estop_rad, now);

    /* 初期化が済んだ最後にWDTを開始（餌やりは run_once が毎周期行う） */
    fc_wdt_start(params->safety.wdt_timeout_ms);
}

/* ---------------------------------------------------------------- 小さな部品 */

/** アームした周期（STANDBY→FLIGHT）の初期化。 */
static void on_arm(control_loop_t *c, const attitude_state_t *as)
{
    attitude_set_bias_locked(&c->att, true);   /* バイアスはアーム時に固定（rev22） */
    c->pid_ctrl.ops->reset(c->pid_ctrl.self);  /* 積分はアーム時に0（rev22） */
    guard_reset(&c->guard);                    /* 新しい飛行: 取り上げを解除 */
    if (c->selected_controller == CONTROLLER_POLICY && policy_available(c)) {
        c->active_controller = CONTROLLER_POLICY;
        c->policy_ctrl.ops->reset(c->policy_ctrl.self);
    } else {
        /* 方策が選ばれても使えないときはPID（今は方策の実装が無いのでこちら） */
        c->active_controller = CONTROLLER_PID;
    }
    descent_init(&c->descent);                 /* ホバリング推力の平均もこの飛行で数え直し */
    c->prev_output = zero_output();
    c->pid_shadow_out = zero_output();
    c->out = zero_output();
    c->yaw_ref_att = as->angle.yaw_rad;
    c->yaw_ref_pkt = finite_or_zero(c->target.yaw);
}

/** 飛行中の人の指示から目標姿勢と目標推力を作る。 */
static void make_pilot_target(control_loop_t *c, const attitude_state_t *as)
{
    (void)as;
    const params_t *p = c->params;
    const control_packet_t *t = &c->target;
    /* 提案: 目標のロール・ピッチは見張り役の傾きしきい値(30°)まで。NaNは0 */
    const float lim = p->guard.tilt_takeover_rad;
    c->target_att.roll_rad  = clampf(finite_or_zero(t->roll), -lim, lim);
    c->target_att.pitch_rad = clampf(finite_or_zero(t->pitch), -lim, lim);
    /* 提案: ヨーはアーム時を基準にした差で目標を作る（PC側の絶対ヨーと機体のヨー(起動時0)がずれていてもよい） */
    c->target_att.yaw_rad = attitude_wrap_pi(c->yaw_ref_att + (finite_or_zero(t->yaw) - c->yaw_ref_pkt));
    c->target_thrust_g = clampf(finite_or_zero(t->throttle), 0.0f, 1.0f) * p->airframe.throttle_full_thrust_g;
}

static void build_log(control_loop_t *c, const attitude_state_t *as)
{
    log_record_t *r = &c->log;
    memset(r, 0, sizeof(*r));
    /* r->seq と dropped_total は ipc_log_push が付ける（ステップ④） */
    r->cycle_start_us = c->timing.start_us;
    r->wake_latency_us = c->timing.wake_latency_us;
    r->proc_us = c->timing.proc_us;
    r->policy_us = c->policy_us;
    r->battery_mv = c->batt.millivolt;
    r->int1_missed = c->timing.int1_missed;
    r->fifo_words = (uint8_t)(c->batch.words_read > 255u ? 255u : c->batch.words_read);
    r->state = (uint8_t)c->fsm.state;
    r->estop_reason = (uint8_t)c->fsm.estop_reason;
    r->descent_reason = (uint8_t)c->fsm.descent_reason;
    r->active_controller = (uint8_t)(c->policy_used ? CONTROLLER_POLICY : CONTROLLER_PID);
    r->takeover_reason = (uint8_t)c->guard.reason;
    uint8_t fl = 0;
    if (c->motors_on) fl |= LOG_FLAG_MOTORS_ON;
    if (c->mix.saturated && c->motors_on) fl |= LOG_FLAG_MIXER_SATURATED;
    if (c->batch.fifo_overrun) fl |= LOG_FLAG_FIFO_OVERRUN;
    if (c->batch.truncated) fl |= LOG_FLAG_FIFO_TRUNCATED;
    if (c->policy_used) fl |= LOG_FLAG_POLICY_ACTIVE;
    r->flags = fl;
    r->attitude[0] = as->angle.roll_rad; r->attitude[1] = as->angle.pitch_rad; r->attitude[2] = as->angle.yaw_rad;
    r->rate[0] = as->rate_rps.x; r->rate[1] = as->rate_rps.y; r->rate[2] = as->rate_rps.z;
    r->target_att[0] = c->target_att.roll_rad; r->target_att[1] = c->target_att.pitch_rad;
    r->target_att[2] = c->target_att.yaw_rad;
    r->target_thrust = c->target_thrust_g;
    r->output[0] = c->out.thrust_g;
    for (int i = 0; i < 3; i++) r->output[1 + i] = c->out.torque_g[i];
    for (int i = 0; i < FC_NUM_MOTORS; i++) r->duty[i] = c->duty[i];
}

/* ---------------------------------------------------------------- 1周期 */

bool control_loop_run_once(control_loop_t *c, const imu_int1_event_t *ev)
{
    const params_t *p = c->params;
    const uint64_t now = fc_clock_now_us64();

    /* ---- 周期の開始 ---- */
    uint32_t ts, delta;
    if (ev != NULL) {
        ts = ev->timestamp_us;
        delta = (c->timing.cycle_index == 0) ? 1u : (uint32_t)(ev->count - c->last_int1.count);
        c->last_int1 = *ev;
    } else {
        ts = (uint32_t)now;   /* INT1が無い周期: 開始時刻は今 */
        delta = 1u;
    }
    cycle_timing_begin(&c->timing, ts, delta);
    c->imu_stalled = (ev == NULL);

    /* ---- TODO(④): ipc_cmd_pop() ×最大 IPC_CMD_MAX_PER_CYCLE → params_set_by_id / policy_stage_chunk /
     *      control_loop_select_controller → ipc_reply_push()。変更系は control_loop_params_change_allowed() が
     *      true（STANDBY）のときだけ受け付ける。 ---- */

    /* ---- IMU（時刻が大事なので最初に読む） ---- */
    c->batch.count = 0;
    c->batch.words_read = 0;
    c->batch.fifo_overrun = false;
    c->batch.truncated = false;
    bool read_ok = false;
    if (ev != NULL) {
        if (imu_read_fifo(&c->batch) == IMU_OK) {
            read_ok = true;
            c->imu_fail_count = 0;
            /* 提案: FIFOをウォーターマーク未満まで読み切ったのにピンがHighのまま = INT1の張り付き。
             * 本物のFIFO_TH線はこの時点でLowのはず。取りこぼし（words_read >= WTM）は数えない */
            if (imu_int1_pin_is_high() && c->batch.words_read < c->fifo_wtm_words) {
                if (c->int1_stuck_count < UINT32_MAX) c->int1_stuck_count++;
                if (c->int1_stuck_count >= FC_LOOP_INT1_STUCK_LIMIT && !c->int1_stuck) {
                    c->int1_stuck = true;   /* 再起動までラッチ */
                    c->stats.int1_stuck++;
                }
            } else {
                c->int1_stuck_count = 0;
            }
            if (c->batch.fifo_overrun) c->stats.overruns++;
            if (c->batch.truncated) c->stats.truncs++;
            imu_mount_apply(&p->imu_mount, &c->batch);   /* 基板の軸 → 機体の軸 */
            attitude_update_batch(&c->att, &c->batch, c->dt_sample);
        } else {
            c->batch.count = 0;
            c->batch.words_read = 0;
            c->imu_fail_count++;
            c->stats.read_errors++;
        }
    }
    const attitude_state_t *as = attitude_get(&c->att);
    const float tilt = as->valid ? attitude_tilt_rad(as) : 0.0f;
    /* 提案: サンプルが0個の周期は制御を進めない（古いデータでdt 8個ぶん進めない） */
    const bool have_samples = read_ok && c->batch.count > 0u;
    const float ctrl_dt = have_samples ? c->dt_sample * (float)c->batch.count : 0.0f;
    if (!have_samples && ev != NULL) {
        c->stats.no_sample++;
        if (c->no_sample_count < UINT32_MAX) c->no_sample_count++;
    } else if (have_samples) {
        c->no_sample_count = 0;
    }

    /* ---- 電池（数周期に1回） ---- */
    if ((c->timing.cycle_index % FC_LOOP_BATT_EVERY_N) == 0u) {
        battery_update(&c->batt, &p->battery, (uint32_t)(now / 1000u));
    }
    bool battery_low = c->batt.low;
    c->batt_v_used = c->batt.voltage_v;
    if (c->bench_no_battery) {
        battery_low = false;
        c->batt_v_used = FC_LOOP_BENCH_BATT_V;
    }

    /* ---- メールボックスと旗 ---- */
    {
        const uint32_t seq = mailbox_seq();
        if (seq != 0u && seq != c->last_mb_seq) {
            /* 読み直しが尽きたら(false)前回の値のまま。seq を更新しないので次の周期で再試行 */
            if (mailbox_read(&c->target)) {
                c->last_mb_seq = seq;
                c->last_new_packet_us = now;
                c->have_packet = true;
            }
        }
    }
    const bool comm_lost = c->have_packet &&
        (now - c->last_new_packet_us) > (uint64_t)p->safety.comm_timeout_ms * 1000u;
    const bool comm_connected = c->have_packet && !comm_lost;
    /* 旗は「つながっている間のパケット」だけを見る。途絶中の古い旗でアームさせない */
    const uint32_t pkt_flags = comm_connected ? c->target.flags : 0u;
    const bool arm_flag = (pkt_flags & CTRL_FLAG_ARM_REQ) != 0u;
    const bool arm_edge = arm_flag && !c->prev_arm_flag;
    c->prev_arm_flag = arm_flag;   /* 状態に関わらず毎周期更新: 待機に着く前に押されていた旗では、アームしない */

    ipc_flags_t *fl = ipc_flags();
    const bool disarm_pending = ipc_flags_disarm_pending(fl);
    const bool estop_req = ipc_flags_estop_requested(fl) ||
                           (c->have_packet && (c->target.flags & CTRL_FLAG_ESTOP) != 0u);
    const bool disarm_req = disarm_pending || (pkt_flags & CTRL_FLAG_DISARM) != 0u;
    const bool human_takeover = (pkt_flags & CTRL_FLAG_TAKEOVER_PID) != 0u;

    /* ---- 降下中なら着地・時間切れを先に判定（今周期の姿勢で） ---- */
    const flight_state_t prev_state = c->fsm.state;
    if (prev_state == FS_EMERGENCY_DESCENT) {
        descent_update(&c->descent, &p->descent, as, now, &c->desc_tgt);
    }

    /* ---- 状態機械 ---- */
    flight_fsm_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.boot_imu_ok = c->boot_imu_ok;
    in.wdt_reboot = c->wdt_reboot;
    /* 提案: INT1張り付きの専用の理由は作らず IMU途絶 に含める */
    in.imu_timeout = (ev == NULL) || (c->imu_fail_count >= FC_LOOP_IMU_FAIL_LIMIT) || c->int1_stuck ||
                     (c->no_sample_count >= FC_LOOP_NO_SAMPLE_LIMIT);
    in.comm_connected = comm_connected;
    in.comm_lost = comm_lost;
    in.arm_rising_edge = arm_edge;
    /* 提案: アーム条件に「gyro バイアスが求まっている」「電池が低下していない」を足す
     * （固定するバイアスが未推定のまま飛ばない／アームした途端に降下に入らない） */
    in.level_and_zero_throttle = as->valid && attitude_bias_ready(&c->att) && !battery_low &&
        (tilt <= p->safety.arm_level_tol_rad) &&
        (finite_or_zero(c->target.throttle) <= p->safety.arm_throttle_tol);
    in.disarm_request = disarm_req;
    in.estop_request = estop_req;
    in.battery_low = battery_low;
    in.tilt_rad = tilt;
    in.landed = (prev_state == FS_EMERGENCY_DESCENT) && c->desc_tgt.landed;
    in.descent_timeout = (prev_state == FS_EMERGENCY_DESCENT) && c->desc_tgt.timed_out;
    flight_fsm_step(&c->fsm, &in, p->safety.tilt_estop_rad, now);
    const flight_state_t st = c->fsm.state;
    if (disarm_pending) ipc_flags_disarm_acknowledge(fl);   /* 今周期の判断で処理した */

    /* ---- 遷移した周期の後始末 ---- */
    if (prev_state != st) {
        if (prev_state == FS_STANDBY && st == FS_FLIGHT) {
            on_arm(c, as);
        } else if (prev_state == FS_FLIGHT && st == FS_STANDBY) {
            attitude_set_bias_locked(&c->att, false);   /* 待機に戻ったらバイアス更新を再開 */
        } else if (st == FS_EMERGENCY_DESCENT) {
            descent_begin(&c->descent, as, now);
            descent_update(&c->descent, &p->descent, as, now, &c->desc_tgt);
        }
    }

    /* ---- 制御器 ---- */
    c->mix.saturated = false;
    for (int i = 0; i < FC_NUM_MOTORS; i++) c->duty[i] = 0.0f;
    /* 提案: 飛行中でもスロットルが idle_throttle 以下なら回さない（状態は飛行のまま）。
     * 地上でアームしたまま傾いていると、P項だけでモーターが回ってしまうため。降下中は機体が回すので対象外 */
    const bool idle = (st == FS_FLIGHT) &&
                      (finite_or_zero(c->target.throttle) <= p->safety.idle_throttle);
    const bool flying = flight_fsm_motors_allowed(st) && !idle;
    /* 提案: サンプルが0個の周期は制御を進めず前回の指令を保つ（PID・方策・見張り役・hover追従は呼ばない）。
     * policy_used は前周期の値のまま、policy_us は0 */
    const bool hold = flying && !have_samples;
    c->policy_us = 0;
    if (!hold) c->policy_used = false;

    if (flying) {
        if (st == FS_FLIGHT) {
            make_pilot_target(c, as);
        } else {
            c->target_att = c->desc_tgt.target_attitude;
            c->target_thrust_g = c->desc_tgt.target_thrust_g;
        }

        if (!hold) {
            controller_input_t cin;
            controller_input_make(as, &c->target_att, c->target_thrust_g, &c->prev_output, &cin);

            /* PIDは毎周期回す: 降下中は本番、方策使用中は影（取り上げ時の積分が冷えない。architecture.md #13） */
            controller_output_t pid_out;
            if (!c->pid_ctrl.ops->step(c->pid_ctrl.self, &cin, ctrl_dt, &pid_out)) pid_out = zero_output();
            c->pid_shadow_out = pid_out;
            c->out = pid_out;

            if (st == FS_FLIGHT && c->active_controller == CONTROLLER_POLICY && policy_available(c)) {
                bool take = guard_taken_over(&c->guard) ||
                            guard_check_pre(&c->guard, &p->guard, as, human_takeover);
                if (!take) {
                    controller_output_t pol_out = zero_output();
                    const uint32_t t0 = fc_clock_now_us32();
                    const bool ok = c->policy_ctrl.ops->step(c->policy_ctrl.self, &cin, ctrl_dt, &pol_out);
                    const uint32_t el = (uint32_t)(fc_clock_now_us32() - t0);
                    c->policy_us = (uint16_t)(el > 65535u ? 65535u : el);
                    if (!ok) {
                        guard_force_takeover(&c->guard, TAKEOVER_POLICY_FAIL);
                        take = true;
                    } else {
                        /* 張りつきの判定: 推力は今の電池での最大、トルクはPIDの内側の出力上限（提案） */
                        float tq_max = 0.0f;
                        for (int i = 0; i < 3; i++) {
                            if (p->pid.rate[i].out_limit > tq_max) tq_max = p->pid.rate[i].out_limit;
                        }
                        take = guard_check_post(&c->guard, &p->guard, &pol_out,
                                                thrust_model_max_g(&p->airframe, c->batt_v_used), tq_max, el);
                        if (!take) {
                            c->out = pol_out;   /* 取り上げでなければ方策の出力を使う */
                            c->policy_used = true;
                        }
                    }
                }
                /* take のときは c->out は影PIDの出力のまま（この周期から差し替え） */
            }
            c->prev_output = c->out;   /* 前回出力は実際に指令した値 */
            if (st == FS_FLIGHT) descent_track_hover(&c->descent, c->out.thrust_g);
        }   /* !hold。hold のときは c->out（= prev_output）をそのままミキサーへ */

        const float vmax = thrust_model_max_g(&p->airframe, c->batt_v_used);
        mixer_mix(&p->airframe, &c->out, vmax, &c->mix);
        for (int i = 0; i < FC_NUM_MOTORS; i++) {
            c->duty[i] = thrust_model_to_duty(&p->airframe, c->mix.motor_g[i], c->batt_v_used);
        }

        if (c->motor_output_enabled) {
            float lim[FC_NUM_MOTORS];
            for (int i = 0; i < FC_NUM_MOTORS; i++) {
                lim[i] = c->duty[i] > c->motor_duty_max ? c->motor_duty_max : c->duty[i];
            }
            motor_pwm_set(lim);
        } else {
            motor_pwm_stop();   /* 試験用: 計算だけしてピンには出さない */
        }
        c->motors_on = true;
    } else {
        /* 停止状態（BOOT / WAIT_COMM / STANDBY / ESTOP）と、飛行中のスロットル0付近。毎周期止める（冪等） */
        c->target_att = (euler_t){ 0.0f, 0.0f, 0.0f };
        c->target_thrust_g = 0.0f;
        c->out = zero_output();
        c->prev_output = c->out;
        motor_pwm_stop();
        c->motors_on = false;
    }

    /* ---- 周期の終了（ここまでが proc_us。ログ組み立てとWDTは含めない） ---- */
    cycle_timing_end(&c->timing);
    c->stats.cycles++;
    if (c->timing.proc_us > LOOP_BUDGET_US) c->stats.over_budget++;

    build_log(c, as);
    /* TODO(④): ipc_log_push(&c->log);（満杯なら捨てて数える。待たない） */

    fc_wdt_feed();   /* 非常停止中も毎周期。止まったら再起動して非常停止に入る */
    c->stats.wdt_feeds++;

    /* ---- INT1の取りこぼし確認（architecture.md #10/#12） ---- */
    /* 読み出しに失敗した周期は急いで読み直さない（失敗を高速で繰り返さず、待ちで時間基準を保つ） */
    /* INT1張り付きがラッチされたら読み直さない（空読みのビジー回りを止める） */
    c->int1_pending = (ev != NULL) && read_ok && !c->int1_stuck && imu_int1_pin_is_high();
    if (c->int1_pending) c->stats.miss++;
    return c->int1_pending;
}

/* ---------------------------------------------------------------- 実機のループ */

void control_loop_step(control_loop_t *c)
{
    imu_int1_event_t ev = c->last_int1;
    bool got;
    if (c->int1_pending) {
        /* 読み終えてもピンがHighだった: 立ち上がりは来ないので待たずに読む。開始時刻は今 */
        got = true;
        ev.timestamp_us = fc_clock_now_us32();
    } else {
        got = imu_wait_int1(FC_INT1_TIMEOUT_US, &ev);
        if (!got) {
            c->stats.timeouts++;
            /* 提案: 張り付きラッチ後は救出しない（空読みになるだけ）。run_once(NULL) で5ms待ちを時間基準にする。
             * 待ちが立ち上がりを返したときは通常どおり run_once(&ev) を呼ぶ（ラッチが imu_timeout を保つ） */
            if (!c->int1_stuck && imu_int1_pin_is_high()) {
                /* ピンはHighのまま = 立ち上がりを取りこぼした。読んで救出する（INT1自体は生きている） */
                c->stats.rescues++;
                got = true;
                ev = c->last_int1;
                ev.timestamp_us = fc_clock_now_us32();
            }
        }
    }
    if (got) {
        control_loop_run_once(c, &ev);
    } else {
        /* INT1が来ずピンもLow = IMU途絶。モーターを止める経路は run_once(NULL) の状態機械に任せ、
         * 5ms待ちが時間基準になってログとWDTの餌やりを続ける（#19） */
        control_loop_run_once(c, NULL);
    }
}

void control_loop_run(control_loop_t *c)
{
    for (;;) control_loop_step(c);
}

/* ---------------------------------------------------------------- 待機中の操作 */

bool control_loop_select_controller(control_loop_t *c, controller_id_t id)
{
    if (c->fsm.state != FS_STANDBY) return false;
    if (id != CONTROLLER_PID && id != CONTROLLER_POLICY) return false;
    c->selected_controller = id;
    c->active_controller = CONTROLLER_PID;
    guard_reset(&c->guard);
    return true;
}

void control_loop_set_policy(control_loop_t *c, controller_t handle)
{
    c->policy_ctrl = handle;
}

bool control_loop_params_change_allowed(const control_loop_t *c)
{
    return c->fsm.state == FS_STANDBY;
}
