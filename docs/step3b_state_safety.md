# ステップ③b 状態遷移・安全（flight_state / guard / emergency_descent / WDT / control_loop）

設計メモ5章の状態機械と安全系を作り、1周期まるごと（`control_loop_run_once`）をホストPCで試験できるようにした。
設計の正本は設計メモ（5章）。メモにないことは「提案」と書いた。数値はすべて仮で、試験台・飛行試験で決める。
ステップ④（コア間リングバッファ、本物の mailbox / ipc_flags、ログ送信、コマンド処理）はまだ。そこに入る場所には `TODO(④)` を残してある。

## 1. 作ったもの

| ファイル | 役割 |
| --- | --- |
| `firmware/src/flight_state.c` | 状態6つの遷移（入力の構造体 → 次の状態。純粋関数）、非常停止のラッチ、理由の記録 |
| `firmware/src/guard.c` | PIDの見張り役。取り上げの判定とラッチ |
| `firmware/src/emergency_descent.c` | 降下の目標推力・目標姿勢、着地検出、時間切れ |
| `firmware/src/control_loop.c` | 1周期の流れ（純粋C。SDKには触れず、ポートのヘッダ越しにだけ呼ぶ）、実機のループ |
| `firmware/src/port_pico/clock_wdt_pico.c` | WDTの本実装（`watchdog_enable` / `watchdog_update` / `watchdog_enable_caused_reboot`） |
| `firmware/src/params.c` | 安全系しきい値の検査を追加（`params_validate`） |
| `firmware/apps/loop_bench.c` | ブレッドボード試験アプリ（キーでPC役を演じる） |
| `firmware/src/mailbox.c` | メールボックスの本物（シーケンスロック、純粋C11 `<stdatomic.h>`）。オーナー実装 `mail_box.c` の考え方を取り込み、`__dmb()` は acquire/release のフェンスに置き換えた。読み直しは上限16回で、尽きたら `false`（**提案**。owner版は無限に待つ） |
| `firmware/src/ipc_flags.c` | 旗の本物（estop・disarm_req・disarm_ack の3語、書く人は各1人）。実体と `ipc_flags()` もここ |
| `firmware/test/` | 偽物（`fake_port.c`: IMU・PWM・時計・WDT・ADC、mailbox・ipc_flags は本物をそのままリンク）と、`test_fsm.c` `test_guard_descent.c` `test_ipc.c` `test_loop.c`。コア1の役は試験が演じる |

ヘッダの変更（`control_loop.h` ほか）:
- `control_loop_run_once` は `bool` を返す（INT1ピンが読み終えてもHigh = 取りこぼしなら true）。`ev==NULL` は「IMU途絶の周期」。
- `control_loop_step`（実機の1ステップ）を公開。`control_loop_run` はこれを繰り返すだけ。ブレッドボードアプリが周期の合間に処理を挟むため。
- `control_loop_select_controller` / `control_loop_set_policy` / `control_loop_params_change_allowed` を追加（コマンド処理のフック）。
- `control_loop_t` に統計・ログ・試験用スイッチ（`motor_output_enabled`、`motor_duty_max`、`bench_no_battery`。既定は本番の値）を追加。
- `guard.h`: 取り上げ理由 `TAKEOVER_POLICY_FAIL` と `guard_force_takeover`、`takeover_reason_name` を追加。`emergency_descent.h`: `descent_t` に内部の状態を追加。
- `flight_state.h`: `ESTOP_INTERNAL_ERROR` と `descent_reason_name` を追加。

## 2. 状態遷移表（実装どおり）

各状態で上の行から順に評価する。「提案」は設計メモにない決め。

| 状態 | 条件 | 次の状態（理由） |
| --- | --- | --- |
| BOOT | WDT起因の再起動 | ESTOP（wdt_reboot） |
| BOOT | IMU初期化失敗 | ESTOP（imu_init_fail） |
| BOOT | 操作者の非常停止 | ESTOP（operator） |
| BOOT | 上のどれでもない | WAIT_COMM |
| 任意（BOOT以外） | 操作者の非常停止（`ipc_flags` または パケットの旗） | ESTOP（operator） |
| WAIT_COMM | IMU途絶（**提案**） | ESTOP（imu_timeout） |
| WAIT_COMM | 通信接続（有効なパケットを受け、途絶していない） | STANDBY |
| STANDBY | IMU途絶（**提案**） | ESTOP（imu_timeout） |
| STANDBY | 通信途絶（**提案**、architecture #20） | WAIT_COMM |
| STANDBY | アーム旗の0→1 かつ 水平・スロットル0 かつ ディスアーム要求なし（**提案**） | FLIGHT |
| FLIGHT | IMU途絶 | ESTOP（imu_timeout） |
| FLIGHT | 傾き > 60°（NaNも） | ESTOP（tilt_exceeded） |
| FLIGHT | ディスアーム | STANDBY |
| FLIGHT | 通信途絶 | DESCENT（comm_lost） |
| FLIGHT | 電池低下 | DESCENT（battery_low） |
| DESCENT | IMU途絶（**提案**） | ESTOP（imu_timeout） |
| DESCENT | 傾き > 60°（**提案**） | ESTOP（tilt_exceeded） |
| DESCENT | 着地検出 | ESTOP（landed） |
| DESCENT | 時間切れ | ESTOP（descent_timeout） |
| ESTOP | 何があっても | 抜けない（電源入れ直しのみ） |

- 降下の原因（`descent_reason`）は非常停止に入ったあとも残り、ログに両方出る（#5）。通信途絶と電池低下が同時なら通信途絶を記録。
- 降下に入ったら人には返さない（通信が戻っても、ディスアーム旗が来ても降下のまま。**提案**: 止めたいときは非常停止）。
- アーム条件は、メモの「水平・スロットル0」に加えて**提案**で「ジャイロのバイアスが求まっている」と「電池が低下していない」を足した。
  固定するバイアスが未推定のまま飛ばさない／アームした途端に降下に入らないため。
- アームの辺（0→1）は、状態に関係なく毎周期検出して消費する。接続前から立っていた旗や、非常停止中に立てた旗では、あとでアームしない。
  通信途絶中のパケットの旗は0として扱う。
- ディスアームは `ipc_flags` のカウンタ（`disarm_req != disarm_ack`）とパケットの旗の両方。カウンタは毎周期、処理したら `acknowledge` する
  （降下中のものも含めて消費し、あとで効いてしまうのを防ぐ）。

## 3. 1周期の流れ（実装）

architecture.md 3章の順序のうち、**IMU読み出しを最初にした**（時刻が大事なため。メモの「ループの最初」のコマンド処理は④で入るので、IMUの前に置くかは④で決める）。

1. `cycle_timing_begin`
2. （TODO(④) `ipc_cmd_pop` → パラメータ・制御器の選択。受付は `control_loop_params_change_allowed()` が true のときだけ）
3. `imu_read_fifo` → `imu_mount_apply` → `attitude_update_batch`（バッチ全サンプル）
4. `battery_update`（8周期に1回）
5. `mailbox_seq` / `mailbox_read`（新しい seq なら目標と `last_new_packet_us` を更新）、`ipc_flags_*`
6. 降下中なら `descent_update`（着地・時間切れ）
7. `flight_fsm_step`。遷移した周期の後始末（アーム: バイアス固定・積分0・guard解除・制御器の確定、降下開始: `descent_begin`）
8. FLIGHT/DESCENT のとき: 目標を作る → PIDを常に1回回す（降下では本番、方策使用中は影）→ 方策使用中なら `guard_check_pre` → 方策 → `guard_check_post` → 取り上げなら影PIDの出力
9. `thrust_model_max_g` → `mixer_mix` → `thrust_model_to_duty` ×4 → `motor_pwm_set`。停止状態は毎周期 `motor_pwm_stop`
10. `cycle_timing_end`（ここまでが `proc_us`）→ ログ（`log_record_t`）を組み立て → （TODO(④) `ipc_log_push`）→ `fc_wdt_feed`
11. `imu_int1_pin_is_high()` を確認し、戻り値と `int1_pending` に入れる

`control_loop_step`（実機）: 取りこぼしが残っていれば待たずに読む。無ければ `imu_wait_int1(5ms)`。タイムアウトでもピンがHighなら読んで救出（RESC）。
ピンもLowなら IMU 途絶として `run_once(c, NULL)`。この場合は 5ms の待ちが時間基準になり、非常停止中でもログ作成とWDT餌やりが続く（#19）。
読み出しに失敗した周期は、ピンがHighでも急いで読み直さない（失敗を高速で繰り返さず、待ちで時間基準を保つ）。

## 4. 個別の仕様と数値（すべて仮）

### 通信
- 「つながった」= 最初の有効パケット（#20）。途絶 = 新しい seq が `comm_timeout_ms`（500ms）来ない。
- 目標: ロール・ピッチは見張り役の傾きしきい値（30°）まで、NaNは0（**提案**）。スロットル(0..1) × `throttle_full_thrust_g`（28g）= 目標推力。
- ヨー目標は**提案**で「アーム時の機体のヨー + (パケットのヨー − アーム時のパケットのヨー)」。PCの絶対ヨーと機体のヨー（起動時0）がずれていても動く。

### 見張り役（guard）
- 方策で飛んでいる間だけ働く。取り上げの条件: 傾き > 30°、角速度の大きさ > 8rad/s、人の指示（`CTRL_FLAG_TAKEOVER_PID`）、出力がNaN/Inf、
  上限の95%以上の張りつきが50周期続く、方策の計算が600µs超、方策の `step` が失敗（重み未ロードなど。**提案**）。
- 一度取り上げたら、その飛行の間は戻さない。解除は `guard_reset`（待機での選び直し `control_loop_select_controller` と、アーム時）だけ。
- 張りつきの基準: 推力は今の電池での最大推力、トルクはPID内側の出力上限の最大（**提案**。方策の本来の出力範囲は学習側と決める）。
- PIDは FLIGHT/DESCENT の間、毎周期回す（影）。取り上げの周期から影PIDの出力に差し替える。
- 方策の実装がまだない（`controller_policy.c` なし）ので、`policy_ctrl.ops==NULL` を「使えない」とし、選んでいてもアーム時にPIDにする。
  試験は偽の方策（`controller_t`）を `control_loop_set_policy` で入れて、guard の配線を通している。

### 非常時降下
- 目標推力 = `thrust_ratio`（0.9）× 飛行中の推力のローパス（`DESCENT_HOVER_LPF_ALPHA` = 0.002/周期、約0.5秒。**提案・仮**）。水平、ヨーは降下開始時の値。
- 着地 = 加速度の大きさ > `shock_g`（3g）を見たあと、角速度 < 0.3rad/s かつ | |a| − 1 | < 0.1g が 500ms 続く。衝撃より前の静かさは数えない。動いたら数え直し。
- 時間切れ = 3000ms。
- 注意: 姿勢の `accel_g` は1周期（約8サンプル≒1ms）の平均なので、それより短い衝撃は鈍る。`shock_g` は飛行試験で決める。

### WDT
- `params.safety.wdt_timeout_ms`（既定 **100ms、仮**。パラメータは既にあったのでそのまま使った）。
  1周期は約1ms、IMU途絶の時間基準は5ms。USB の `printf` の詰まりを許す余裕も見て100にした。実機で周期の最悪値を測って詰める。
  `params_validate` は 20ms 未満を拒否（INT1待ち5msより十分長くするため）。
- 開始は `control_loop_init` の最後。餌やりは `run_once` が毎周期1回（非常停止中も）だけ行う。
- 再起動の原因判定は BOOT で `fc_wdt_caused_reboot()`。true なら ESTOP（wdt_reboot）。

### 電池
- 低下確定（`battery.low`）は FLIGHT での降下のきっかけ。電池なしの試験用に `bench_no_battery`（3.8V固定・低下判定なし）を用意した。実機の本番では false のまま。

## 5. 提案（メモにない決め）の一覧

| # | 内容 |
| --- | --- |
| 1 | WAIT_COMM / STANDBY でもIMU途絶は ESTOP（imu_timeout）。飛べない機体を待機させない。地上の一時的な途絶でも電源入れ直しになる欠点がある |
| 2 | DESCENT 中もIMU途絶・傾き>60°は ESTOP。姿勢が分からない・戻せない機体を回し続けない |
| 3 | STANDBY で通信途絶 → WAIT_COMM（#20の案）。再接続で STANDBY に戻る。アームには旗を離して押し直しが必要 |
| 4 | アーム条件に「バイアス確定済み」「電池低下なし」を追加。ディスアーム要求と同時のアームは見送り |
| 5 | 降下中はディスアーム旗を無視（消費はする）。降下は人に返さない。止めるには非常停止旗 |
| 6 | 読み出し失敗が連続3回（`FC_LOOP_IMU_FAIL_LIMIT`）でIMU途絶。1〜2回は古い姿勢で続ける |
| 7 | 目標のロール・ピッチを±30°（見張り役のしきい値）に制限。ヨー目標はアーム時基準の差分 |
| 8 | ホバリング推力の推定は 0.5 秒時定数のローパス。アームで数え直し（飛行開始直後に降下になると低めに出る） |
| 9 | guard の張りつき基準（推力=現在の最大、トルク=PID内側の上限）。方策の step 失敗も取り上げ理由にした |
| 10 | 試験用スイッチ（`motor_output_enabled` `motor_duty_max` `bench_no_battery`）を `control_loop_t` に持つ。既定は本番の値 |
| 11 | `ESTOP_INTERNAL_ERROR`（不正な状態値の安全側の行き先） |
| 12 | `params_validate` に安全系の検査を追加（WDT≥20ms、傾き30°<60° など） |
| 13 | 飛行中でもスロットルが `idle_throttle`（0.05）以下ならモーターを止める（状態は飛行のまま）。地上でアームしたまま傾いていると、P項だけでモーターが回るのを防ぐ。降下中は対象外（Opusレビューで追加） |
| 14 | サンプルが0個（または読み出し失敗）の飛行中・降下中の周期は、PID・方策・見張り役を進めず前回の指令を保つ（古いデータでdt 8個ぶん進めない）。保つのは連続 `FC_LOOP_NO_SAMPLE_LIMIT`（16）周期までで、それを超えたら IMU途絶（`imu_timeout`）として ESTOP。統計 `no_sample` |
| 15 | 読み切って（`words_read` < ウォーターマーク）もINT1がHighの周期が連続 `FC_LOOP_INT1_STUCK_LIMIT`（8）回で「INT1張り付き」をラッチ（再起動まで）。専用の理由は作らず IMU途絶（`imu_timeout`）として ESTOP。ラッチ後は読み直し・救出の読み出しをしない（ビジー回り防止）。統計 `int1_stuck` |

## 6. 設計メモであいまいだった点（どう読んだか）

- 「IMU故障」の範囲: BOOT と FLIGHT にだけ書いてある。待機系・降下中は未記載 → 提案1・2。
- 降下中のディスアームや、降下中に通信が戻った場合 → 未記載。人に返さない（提案5）。
- 「スロットルが0」: 0.05以下（`arm_throttle_tol`）を0とみなす。
- 角速度「大きすぎる」: 3軸の合成の大きさで見る。値 8rad/s は仮。
- ヨー目標が角度か角速度か（パケット）: 3a のとおり角度。基準合わせは提案7。
- 非常停止中も「PCへ報告を続ける」: ログの組み立ては続けているが送信は④。
- WDT再起動のあと、RUNピンのリセット等で原因フラグが残るか → 要確認（電源入れ直しでは消える想定）。
- ディスアームの旗カウンタを `acknowledge` する瞬間に新しい要求が来ると、1回分を取りこぼしうる（本物の実装は④）。パケットの旗は毎回送られるので次の周期で拾える。

## 7. 試験

### 7.1 ホストPC

```sh
make -C firmware/test
```

`ALL PASS: 56 tests, 1503 checks`（purity も OK）が出れば合格。③bで足した試験:

| 対象 | 内容 |
| --- | --- |
| fsm | BOOT / WAIT_COMM / STANDBY / FLIGHT / DESCENT の全遷移と優先順、アーム条件（辺・水平・スロットル0・ディスアーム同時）、操作者の非常停止を全状態から、非常停止は乱数入力5000通りでも抜けない、乱数20万ステップで不変条件（FLIGHTへはSTANDBYのアーム条件からのみ、DESCENTはFLIGHTからのみ、など） |
| guard | 傾き・角速度（境界値、合成、NaN）・人の指示・出力NaN/Inf・計算時間・張りつき（ちょうどN周期、外れたら数え直し、負側）、ラッチ、後の異常で理由を上書きしない、reset |
| descent | 目標推力＝比×推定、ヨー保持、ホバリング推定の収束と異常値の無視、衝撃なしの静かさは着地でない、衝撃→静かさ500ms、動き・加速度のずれで数え直し、バウンド、時間切れ（64bit時刻） |
| ipc | mailbox: 書いて読む・seq は +2 で常に偶数・未書き込みは0・上書き・書き込み途中（奇数）は読み直し上限で失敗して dst を変えず公開 seq も進めない。ipc_flags: estop は立てるだけ、disarm カウンタ（何回でも1回分、処理後の要求は残る、32bit一周）、acknowledge は estop に触れない |
| loop | 起動（PWM0→WDT開始）、IMU無応答・WDT再起動、ライフサイクル（通信待機→待機→アーム→飛行→ディスアーム[カウンタ/パケット旗]→再アーム）、アーム条件、通信途絶→降下→着地→非常停止、降下→時間切れ、電池低下→降下、操作者の非常停止（全状態×旗2種）と抜けないこと、傾き60°超、IMU途絶（タイムアウト・救出・連続失敗・降下中）、INT1取りこぼし、ログと周期時間、PWM出力無効・デューティ上限、偽の方策（使用・取り上げ各原因・ラッチ・選び直し・降下で呼ばない） |

`loop` の全試験で毎周期 ①停止状態では `motor_pwm_set` を呼ばずデューティが0、②WDTは `run_once` ごとにちょうど1回（非常停止中も）を確かめている。

### 7.2 ブレッドボード（loop_bench）

ビルド（PWMは出ない）:

```sh
cmake -S firmware -B build -DPICO_SDK_PATH=<pico-sdk> -DPICO_BOARD=pico2_w
make -C build            # build/loop_bench.uf2
```

書き込みと配線は step2 / step3a と同じ（IMUのみ。モーター・電池は接続しなくてよい）。ターミナルは USB シリアルを開く。

**PWM出力について（安全）**: control_bench と同じ。既定では出さない（状態機械は動き、デューティは計算して表示するが、ピンは0のまま）。
`-DFC_BENCH_PWM_ENABLE=ON` のときだけ出す。

> **警告: PWMを有効にするときは必ずプロペラを外す。** デューティは 0.30 で頭打ち（`FC_BENCH_DUTY_MAX`）。`+` で飛行中にモーターが回る。

電池をつないでいない（ADC 2V未満）と起動時に「電池なし」モードになり、電池低下で降下に入らない（3.8V固定で換算）。電池をつないだ試験では通常どおり判定する。

コマンド（1文字）: `c` 接続 / `s` 送信停止 / `a` アーム押す⇄離す / `d` ディスアーム / `e` 非常停止 / `+` `-` スロットル±0.05 / `0` スロットル0 / `t` 取り上げ旗 / `w` WDT試験 / `h` ヘルプ。
表示は1秒に1行: 状態、estop理由、降下理由、guard、制御器、姿勢、スロットル、推力、デューティ、`proc_us avg/max`、`over`（1064µs超の回数）、`TMO`（INT1待ちタイムアウト）、`RESC`（救出）、`MISS`（取りこぼし）、`ERR`（読み出し失敗）、`STUCK`（INT1張り付きのラッチ回数）、`NOSMP`（サンプル0個で制御を保った周期数）、`WDT_fed`（餌やり回数）、PC役の状態。

手順と合格条件:

| # | 手順 | 合格条件 |
| --- | --- | --- |
| 1 | 起動し、ボードを水平に静止 | `init:` 行が `state=WAIT_COMM estop=none`。毎秒の行が `WAIT_COMM`、`duty` はすべて0、`WDT_fed` が増え続ける |
| 2 | `c` | 1秒以内に `STANDBY` |
| 3 | 起動直後から一度も1秒続けて静止させずに（手で揺らし続けたまま）`c` → `a`（押す）→ `a`（離す） | `STANDBY` のまま（バイアス未確定は見送り）。バイアスは起動後に初めて1秒静止した時点で確定し、以後は揺らしても取り消されない。最後に1秒以上静止させる |
| 4 | `+` でスロットルを上げて `a` → `a`（離す）→ `0` | スロットル>0.05 でのアームは `STANDBY` のまま。ボードを傾けて（10°超）`a` も `STANDBY` のまま |
| 5 | 水平・スロットル0で `a`（押す） | `FLIGHT`。`ctrl=PID`。スロットル0のあいだは傾けても `duty` は0（提案13） |
| 6 | `+` を数回 | スロットルが0.05を超えると `duty` が計算されて表示される（PWM無効ビルドではピンは0）。`T=` と `duty` が増える。傾けると PID が戻す向きに動く（step3a の合格条件3と同じ） |
| 7 | `d` | 1周期で `STANDBY`。`duty` 0。`pc[... arm=0]`（`d` で旗は離した扱いになる）。スロットル0に戻して `a`（押す）で再アームできる。パケットのディスアーム旗は0.3秒だけ立つ（その間はアームしない） |
| 8 | `FLIGHT` で `s`（送信停止） | 約0.5秒後に `DESCENT desc=comm_lost`、`T=` が飛行時の約0.9倍 |
| 9 | 続けて待つ（台上では着地の衝撃が無い） | 約3秒後に `ESTOP estop=descent_timeout desc=comm_lost`、`duty` 0。`c` `a` `d` を押しても `ESTOP` のまま |
| 10 | 電源を入れ直し、`FLIGHT` で `e` | 即 `ESTOP estop=operator`。電源入れ直しまで戻らない（`c` `a` を押しても） |
| 11 | 電源を入れ直し、`STANDBY` で `e`、`WAIT_COMM`（`c` 前）で `e` | どちらも `ESTOP estop=operator` |
| 12 | `FLIGHT` でボードを60°超に傾ける | `ESTOP estop=tilt_exceeded` |
| 13 | `FLIGHT` で `t` | `guard=` は変わらない（方策がなく見張り役が働かない）。方策が入ったら `guard=human` になる |
| 14 | 全項目を通して `proc_us avg/max` | max が 1064µs に余裕をもって収まる（目安: 最大800µs以下、平均はその半分以下）。`over=0` `ERR=0` `STUCK=0` `NOSMP=0`（正常なら0。`STUCK=1` は INT1 が張り付いた印で `ESTOP estop=imu_timeout` になる）。`TMO=0`（IMU途絶の試験をしない限り）。`MISS` `RESC` は数を記録 |
| 15 | WDT試験: `STANDBY` で `w` | 約100ms後に再起動。再接続後の `init:` 行が `state=ESTOP estop=wdt_reboot wdt_reboot=1`。`c` `a` を押しても `ESTOP` のまま。電源入れ直しで `WAIT_COMM` に戻る |
| 16 | IMU途絶（任意）: 通電中にIMUのINT1線を外す、または電源を落とす | `ESTOP estop=imu_timeout`、`duty` 0、行は約5msごとの時間基準で出続け `WDT_fed` が増える（再起動しない） |

WDT試験（15）は停止状態のときだけ受け付ける（`FLIGHT` / `DESCENT` では拒否）ので、PWM有効ビルドでも安全。
loop_bench だけが、`printf` が詰まってもリセットしないよう、表示の前後で `fc_wdt_feed` を呼んでいる（本体は餌やりが `run_once` の1か所だけ）。

### 7.3 実機で確認すること（未測定）
- `proc_us` の実測（方策なしのPIDで）。ホスト試験では時間は測れない。
- 着地検出の `shock_g`、降下の `thrust_ratio`、各しきい値（飛行試験）。
- WDT再起動後に原因フラグが想定どおり判定できること（試験15）。

## 8. ステップ④に残したこと（TODO(④)）

- （mailbox / ipc_flags の本物は③bで前倒しして実装済み。残るのはキュー側: `spsc_ring` `ipc.c`）
- 書く側の割り込み禁止: `src/` はSDKに触れないので、実機でコア1の割り込みの中から `mailbox_write` するなら呼ぶ側が止める（`mailbox.h` に注記）。
- `ipc_log_push(&c->log)`（`control_loop.c` の `build_log` の直後）。ログの `seq` / `dropped_total` は push 側が付ける。
- ループの最初の `ipc_cmd_pop` とコマンド処理（変更系は `control_loop_params_change_allowed()` が true のときだけ）。制御器の選択は `control_loop_select_controller`。
- 非常停止中の「PCへの報告」（ログ送信）。
- 方策の実装（`controller_policy.c`）。入れたら `control_loop_init` で `policy_init` し、`control_loop_set_policy(c, policy_handle(&c->policy))` する。
- `main.c`（`ipc_init` → コア1起動 → `control_loop_init` → `control_loop_run`）。
