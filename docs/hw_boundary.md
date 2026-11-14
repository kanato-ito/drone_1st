# ハード依存の境界（ポート層）

## 1. なぜ作るか

2026-11-08 のオーナー決定: PCのシミュレータは、最初から制御の1周期まるごと（将来の `control_loop_run_once`、ステップ③b）をホスト上でビルドして呼ぶ。
そのため、ハードに依存する部分は **レジスタやSPIの層ではなく、ドライバのAPI（ヘッダ）の層で** リンク時に差し替えられるようにする。
シミュレータ班は、同じヘッダを実装した自前の `.c` を書くだけでよい（Pico SDK は使わない）。

## 2. 4つの規則

1. `firmware/include/*.h` は SDK のヘッダを include しない。
2. `firmware/src/*.c` は純粋C11。Pico SDK を include しない。ホストの gcc でそのままコンパイルできる。
3. `firmware/src/port_pico/*.c` だけが Pico SDK に触る。1ファイルが1つの「ポートのヘッダ」のAPIを実装する。
4. シミュレータが差し替えるのは `port_pico` のファイルだけ。それ以外はそのままリンクする。

規則2は `make -C firmware/test` が確かめる（`purity` の手順で `src/*.c` を全部ホストgccでコンパイルする。SDKのincludeが混ざると失敗する）。

## 3. ポートのAPI

| ヘッダ | 関数 | 実機の実装 | 偽物が守ること | 呼ぶ人・頻度 |
| --- | --- | --- | --- | --- |
| `imu_lsm6dsv16x.h` | `imu_config_default` `imu_init` `imu_whoami_check` `imu_config_readback` `imu_spi_actual_baud_hz` `imu_fifo_flush` `imu_on_int1_edge` `imu_wait_int1` `imu_int1_pin_is_high` `imu_read_fifo` `imu_sample_period_s` | `port_pico/imu_lsm6dsv16x.c` | `imu_wait_int1` はタイムアウトまで待ち、来なければ false。`imu_read_fifo` は `imu_batch_t` を本物と同じ物理単位（gyro は rad/s、accel は g、軸はIMU基板の軸のまま。偽物も並べ替えない。機体の軸への並べ替えは共通コード `src/imu_mount.c` が行う）で埋める。`imu_sample_period_s` は本物のサンプル間隔。`imu_int1_pin_is_high` は false でよい | コア0。待ち・読み出しは1周期に1回（約940Hz）、初期化は起動時 |
| `motor_pwm.h` | `motor_pwm_init` `motor_pwm_set` `motor_pwm_stop` | `port_pico/motor_pwm.c` | `set` で受けたデューティ(0..1、範囲外とNaNは0)を覚える。`stop` で全部0 | コア0。`set` は1周期に1回、`stop` は必要なとき |
| `cycle_timer.h`（時計・WDT） | `fc_clock_now_us64` `fc_clock_now_us32` `fc_wdt_start` `fc_wdt_feed` `fc_wdt_caused_reboot` | `port_pico/clock_wdt_pico.c`（③bでWDTも本実装） | 時計は単調増加（仮想時間でよい）。us32 は us64 の下位32bit。WDTは空でよい（`caused_reboot` は false）。試験の偽物は呼んだ回数と開始時の時間を覚える（`test/fake_port.c`） | 時計は随時（両コア）。`start` は起動の最後に1回、`feed` はコア0、1周期に1回 |
| `batt_adc.h` | `batt_adc_init` `batt_adc_read_raw` | `port_pico/batt_adc_pico.c` | 12bitの生カウント(0..4095)を返す。分圧比などの換算は `battery.c` が行うので偽物はしない | コア0。読み出しは数周期に1回（数µs） |

`cycle_timing_begin/end` は純粋C（`src/cycle_timer.c`）で、時計の関数だけを使う。ポートではない。

## 4. ポートではないもの

`mailbox`、`ipc_flags` は、`<stdatomic.h>` を使う純粋C11で `src/`（`mailbox.c`、`ipc_flags.c`）に**実装済み**（ステップ③bで前倒し）。
`spsc_ring` と `ipc.c` のキュー側はステップ④。
SDKに触らないので、シミュレータは本物をそのままリンクし、コア1の役（パケットを書く、旗を立てる、ログを読む）を自分で演じる。

- `mailbox_read` は `bool` を返す。書き込み途中で読み直しが上限（16回）を超えたら `false`（`dst` は変えない。呼ぶ側は前回の値で続け、次の周期で再試行。提案）。
- `mailbox_write` の割り込み禁止は呼ぶ側の責任（`src/` はSDKに触れない）。
- 旗の実体と `ipc_flags()` は `ipc_flags.c` にある。

## 5. ホストのプロジェクトから使う

`firmware/cmake/fc_sources.cmake` はSDKを呼ばないので、ホストのCMakeから include できる。

```cmake
include(/path/to/drone_1st/firmware/cmake/fc_sources.cmake)   # FC_INCLUDE_DIR, FC_CORE_SOURCES

add_library(fc_core STATIC ${FC_CORE_SOURCES})
target_include_directories(fc_core PUBLIC ${FC_INCLUDE_DIR})

# 自前の偽物（imu, motor_pwm, 時計/WDT, batt_adc）。FC_PORT_PICO_SOURCES は使わない
add_executable(sim sim_main.c sim_imu.c sim_motor.c sim_clock.c sim_batt.c)
target_link_libraries(sim PRIVATE fc_core)
```

偽物の最小の見本は `firmware/test/fake_port.c`（batt_adc と時計）。

## 6. `control_loop_run_once(c, ev)` の方針（ステップ③b、実装済み）

```c
bool control_loop_run_once(control_loop_t *c, const imu_int1_event_t *ev);
```

- 中では待たない。INT1の記録 `ev` は呼ぶ側が渡す。`ev==NULL` は「INT1が来ない（IMU途絶）」周期。
- 戻り値は「読み終えてもINT1ピンがHigh」（取りこぼし）。true なら待たずにすぐもう一度呼ぶ。
- 実機: `control_loop_step`（`control_loop_run` が繰り返す）が `imu_wait_int1` の結果を渡して呼ぶ。タイムアウト・救出もこちら。
- シミュレータ: 仮想時間で作った `ev` を渡して直接呼ぶ（`imu_int1_pin_is_high` は false を返してよい）。
- 中から呼ぶのは `imu_init`（起動時）`imu_read_fifo`、`motor_pwm_*`、`battery_update`、`fc_clock_*`、`fc_wdt_*`、mailbox / ipc_flags だけ。すべてヘッダ越し。
- リンクに必要なもの: 上のポート4つ（imu, motor_pwm, 時計/WDT, batt_adc）に加え、`mailbox_*` `ipc_flags_*` と `ipc_flags()`（ipc.h）。
  本物は `src/mailbox.c` と `src/ipc_flags.c`（実装済み）なので、シミュレータもホスト試験もそのままリンクする。
- 例: `test/test_loop.c` が、ここに書いたとおりの使い方（偽物の IMU を `fake_imu_*` で操作）をしている。

## 7. 制御4モジュールのC/C++切り替え（提案）

オーナー（kanato）が `thrust_model → mixer → controller_pid → attitude` の順にC++へ書き直す。`src/*.c` は参考例として残し、C++版は `src_cpp/<module>.cpp` に置く。
C++版は**既存のヘッダと同じCのインターフェース**（Cリンケージ）を実装するので、他のC側はどちらにもリンクできる。ヘッダは `extern "C"` つき。
C専用（`<stdatomic.h>`）の `ipc_flags.h` `spsc_ring.h`（と、経由する `ipc.h` `control_loop.h`）はC++から include しない。4モジュールはこれらに依存しない。

切り替えはモジュールごと（提案: 1つずつ書き換えるため）。

| 場面 | 指定 |
| --- | --- |
| Pico（CMake） | `-DFC_CPP_MODULES="thrust_model;mixer"` |
| ホスト試験 | `make -C firmware/test CPP_MODULES="thrust_model mixer"` |
| シミュレータ（ホストCMake） | `fc_sources.cmake` を include する前に `set(FC_CPP_MODULES ...)`、または `-DFC_CPP_MODULES=...`。`FC_CORE_SOURCES` が自動で置き換わる |

- 全部C++にするなら `FC_CPP_MODULES="thrust_model;mixer;controller_pid;attitude"`（以前の案 `FC_CONTROL_LANG=CPP` 相当）。
- 使える名前はこの4つだけ。違う名前や `src_cpp/<m>.cpp` が無いときは CMake が `FATAL_ERROR`、`make` が `error` で止まる。
- `fc_sources.cmake` は `FC_CPP_COMPILE_OPTIONS`（`-fno-exceptions -fno-rtti -Wall -Wextra ...`）も出す。ホストCMakeは `target_compile_options(fc_core PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${FC_CPP_COMPILE_OPTIONS}>)` で .cpp にだけ適用し、`project()` に `CXX` を入れること（Pico側は適用済み）。
- `make -C firmware/test` の `cxxheaders` が、`include/*.h`（C専用を除く）をg++で include できることを毎回確かめる。
- 規則は `firmware/src_cpp/README.md`。
