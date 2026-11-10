/**
 * @file control_loop.h
 * @brief コア0の制御ループ本体。INT1待ち → FIFO → 姿勢 → 状態機械 → 制御器 → ミキサー → PWM。
 *
 * ハード依存の境界は docs/hw_boundary.md。呼び出し順は docs/architecture.md の「1周期の流れ」と
 * docs/step3b_state_safety.md を参照。このファイル(と control_loop.c)は純粋C11で、SDKには触れない。
 * INT1の割り込みの中では何もせず（時刻記録のみ）、処理はメインループ側で行う。
 */
#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdint.h>
#include <stdbool.h>
#include "params.h"
#include "ipc.h"
#include "imu_lsm6dsv16x.h"
#include "attitude.h"
#include "controller.h"
#include "controller_pid.h"
#include "controller_policy.h"
#include "guard.h"
#include "emergency_descent.h"
#include "mixer.h"
#include "battery.h"
#include "flight_state.h"
#include "cycle_timer.h"
#include "log_record.h"
#include "control_packet.h"

/** 電池ADCを読む周期数（電圧は遅い）。 */
#define FC_LOOP_BATT_EVERY_N 8u
/** IMUの読み出しが連続でこれだけ失敗したらIMU途絶（提案。1周期≒1ms なので約3ms）。 */
#define FC_LOOP_IMU_FAIL_LIMIT 3u
/**
 * 提案: 読み切ったのにピンがHighのままの周期がこれだけ連続したら INT1 張り付きとして IMU途絶 にする。
 * 空読みの回転では1ms未満で到達し、本物の取りこぼし（words_read >= WTM）では起きない。
 */
#define FC_LOOP_INT1_STUCK_LIMIT 8u
/**
 * 提案: サンプル0個（読み出し失敗を含む）の周期がこれだけ連続したら IMU途絶 にする。
 * 前回の指令を保つ時間に上限を設ける（通常の周期で約17ms）。張り付き以外の原因で
 * データが来ないまま古い指令を出し続けないため。
 */
#define FC_LOOP_NO_SAMPLE_LIMIT 16u
/** 電池なし試験モード(bench_no_battery)で換算に使う電圧 [V]。 */
#define FC_LOOP_BENCH_BATT_V 3.8f

/** 動作の統計（ブレッドボード表示・試験用）。 */
typedef struct {
    uint32_t cycles;       /**< run_once が処理を終えた回数 */
    uint32_t timeouts;     /**< INT1待ちがタイムアウトした回数 */
    uint32_t rescues;      /**< タイムアウトしたがピンがHighで、読んで救出した回数 */
    uint32_t miss;         /**< 読み終えてもINT1がHighのまま（待たずに読み直した）回数 */
    uint32_t read_errors;  /**< imu_read_fifo の失敗回数 */
    uint32_t overruns;     /**< FIFOあふれの回数 */
    uint32_t truncs;       /**< 1回で読み切れなかった回数 */
    uint32_t over_budget;  /**< proc_us が 1064µs を超えた回数 */
    uint32_t wdt_feeds;    /**< fc_wdt_feed を呼んだ回数 */
    uint32_t int1_stuck;   /**< INT1張り付きをラッチした回数（0か1） */
    uint32_t no_sample;    /**< 読めたがサンプル0個（または読み出し失敗）で制御を進めなかった周期（ev あり） */
} control_loop_stats_t;

/** コア0の全状態。静的に1つだけ確保する（mallocなし）。 */
typedef struct {
    const params_t *params;
    cycle_timing_t timing;
    imu_int1_event_t last_int1;
    imu_batch_t batch;
    attitude_estimator_t att;
    battery_state_t batt;
    flight_fsm_t fsm;
    guard_t guard;
    descent_t descent;
    pid_controller_t pid;
    policy_controller_t policy;          /**< 実体（実装は未着手。今は触らない） */
    controller_t pid_ctrl;               /**< PIDのハンドル（init で作る） */
    controller_t policy_ctrl;            /**< 方策のハンドル。ops==NULL なら使えない（既定）。control_loop_set_policy で入れる */
    controller_id_t selected_controller; /**< 飛行指示待機で選んだ方 */
    controller_id_t active_controller;   /**< このアームで実際に使う方（アーム時に確定。方策が使えなければPID） */
    controller_output_t prev_output;     /**< 前回出力（制御器の入力14の最後の4つ） */
    controller_output_t pid_shadow_out;  /**< PIDの出力（方策使用中は影、それ以外は本番） */
    controller_output_t out;             /**< 今周期にミキサーへ渡した出力 */
    control_packet_t target;             /**< 直近のメールボックス内容 */
    uint32_t last_mb_seq;
    uint64_t last_new_packet_us;         /**< 通信途絶の判定用 */
    bool have_packet;                    /**< 一度でも有効なパケットを受けた */
    bool prev_arm_flag;                  /**< 0→1検出用 */
    uint32_t imu_fail_count;             /**< 連続の読み出し失敗数 */

    /* ---- ここから ③b で追加 ---- */
    bool  boot_imu_ok;                   /**< 起動時のimu_init成功 */
    bool  wdt_reboot;                    /**< 起動時にWDT起因と判った */
    float dt_sample;                     /**< サンプル間隔 [s] */
    bool  int1_pending;                  /**< 読み終えてもINT1がHigh: 次は待たずに読む */
    uint32_t fifo_wtm_words;             /**< FIFOウォーターマーク [ワード]（init で imu_config から） */
    uint32_t int1_stuck_count;           /**< 読み切ったのにピンがHighの連続周期数 */
    uint32_t no_sample_count;            /**< サンプル0個（読み出し失敗を含む）の連続周期数 */
    bool  int1_stuck;                    /**< INT1張り付きをラッチ（再起動まで。IMU途絶として非常停止、読み直しもしない） */
    bool  imu_stalled;                   /**< 今周期はIMUのデータなし（INT1が来ずピンもLow） */
    descent_target_t desc_tgt;           /**< 降下の目標（降下中のみ有効） */
    euler_t target_att;                  /**< 今周期の目標姿勢 */
    float   target_thrust_g;             /**< 今周期の目標推力 */
    float   yaw_ref_att;                 /**< アーム時の機体ヨー（パケットのヨーとの基準合わせ） */
    float   yaw_ref_pkt;                 /**< アーム時のパケットのヨー */
    mixer_result_t mix;
    float   duty[FC_NUM_MOTORS];         /**< 計算したデューティ（飛行中以外は0） */
    bool    motors_on;                   /**< 今周期、motor_pwm_set を呼んだ（飛行中・降下中） */
    bool    policy_used;                 /**< 今周期は方策の出力を使った */
    uint16_t policy_us;                  /**< 今周期の方策の計算時間 */
    float   batt_v_used;                 /**< 換算に使った電池電圧 [V] */
    log_record_t log;                    /**< 今周期のログ（ステップ④でキューへ積む） */
    control_loop_stats_t stats;

    /* ---- 試験用の安全スイッチ（実機の本番は触らない。既定のまま） ---- */
    bool  motor_output_enabled;          /**< false なら飛行中でも motor_pwm_stop のまま（デューティは計算する）。既定 true */
    float motor_duty_max;                /**< motor_pwm_set に渡すデューティの上限。既定 1.0 */
    bool  bench_no_battery;              /**< true なら電池を無視（3.8V固定・低下判定なし）。電池なしのブレッドボード用。既定 false */
} control_loop_t;

/**
 * 全モジュールを初期化する（IMU・PWM・ADC・WDT・状態機械）。
 * 呼ぶコア: コア0。頻度: 起動時に1回。ipc_init() の後。IMU失敗や
 * WDT再起動なら状態を非常停止にして戻る（ループは動かし続ける）。
 * 最初に motor_pwm_init → motor_pwm_stop を行い、最後にWDTを開始する。
 * 試験用スイッチ(motor_output_enabled など)は既定値で返る。変えるなら init の後、ループの前に。
 */
void control_loop_init(control_loop_t *c, const params_t *params);

/**
 * 1周期ぶんの処理を行う。ここでは待たない。
 * INT1の記録 ev は呼び出し側が渡す: 実機では control_loop_step が imu_wait_int1 の結果を、
 * シミュレータは仮想時間で作った記録を渡して直接呼ぶ（待ちも実時間も要らない）。
 * ev==NULL は「INT1が来ない（IMU途絶）」周期: 読み出しはせず、IMU途絶として状態機械に渡す。
 * 非常停止中も同じ（モーター停止のまま、ログ作成とWDTの餌やりを続ける。architecture.md #19）。
 *
 * 中では imu_read_fifo（直後に imu_mount_apply で基板の軸→機体の軸へ）、motor_pwm_*、battery_update、
 * fc_clock_*、fc_wdt_feed、mailbox / ipc_flags を、それぞれのヘッダ越しにだけ呼ぶ（SDKには触らない）。
 *
 * @return INT1ピンが読み終えてもまだHigh（立ち上がりを取りこぼした）なら true。
 *         呼び出し側は待たずにすぐもう一度 run_once を呼ぶ（architecture.md #10/#12）。
 *         同じ値が c->int1_pending にも入る。
 * 呼ぶコア: コア0。頻度: 約940Hz。
 */
bool control_loop_run_once(control_loop_t *c, const imu_int1_event_t *ev);

/**
 * 実機用の1ステップ: （取りこぼしが無ければ）imu_wait_int1(5ms) → 必要なら救出 → run_once。
 * タイムアウトでピンもLowならIMU途絶として run_once(c, NULL)（5msごとの時間基準になる）。
 * ブレッドボードアプリが周期の合間に別の処理を挟めるよう公開している。
 * 呼ぶコア: コア0。頻度: 1周期に1回。
 */
void control_loop_step(control_loop_t *c);

/**
 * メインループ（実機用）。control_loop_step を繰り返す。戻らない。
 * 呼ぶコア: コア0（main から）。頻度: 起動時に1回呼んだら戻らない。
 */
void control_loop_run(control_loop_t *c);

/**
 * 飛行指示待機で使う制御器を選ぶ（guard も解除）。待機(STANDBY)以外では false で何もしない。
 * 方策が使えない（ハンドル未設定）ときもアーム時にPIDへ落ちるだけで、ここは受け付ける。
 * 呼ぶコア: コア0。ステップ④のコマンド処理（CMD_SELECT_CONTROLLER）から。
 */
bool control_loop_select_controller(control_loop_t *c, controller_id_t id);

/** 方策のハンドルを入れる（NULL の ops で外す）。コア0、起動時か待機中。方策の実装が入るまでは試験が使う。 */
void control_loop_set_policy(control_loop_t *c, controller_t handle);

/**
 * パラメータや重みの変更を受け付けてよいか（飛行指示待機のときだけ true）。
 * TODO(④): ループの最初で ipc_cmd_pop し、変更系コマンドの前にこれを見て、
 *          false なら CMD_RESULT_REJECTED_STATE を返信する。
 */
bool control_loop_params_change_allowed(const control_loop_t *c);

#endif /* CONTROL_LOOP_H */
