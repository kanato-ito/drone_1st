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
| `cycle_timer.h`（時計・WDT） | `fc_clock_now_us64` `fc_clock_now_us32` `fc_wdt_start` `fc_wdt_feed` `fc_wdt_caused_reboot` | `port_pico/clock_wdt_pico.c` | 時計は単調増加（仮想時間でよい）。us32 は us64 の下位32bit。WDTは空でよい（`caused_reboot` は false） | 時計は随時（両コア）。`feed` はコア0、1周期に1回 |
| `batt_adc.h` | `batt_adc_init` `batt_adc_read_raw` | `port_pico/batt_adc_pico.c` | 12bitの生カウント(0..4095)を返す。分圧比などの換算は `battery.c` が行うので偽物はしない | コア0。読み出しは数周期に1回（数µs） |

`cycle_timing_begin/end` は純粋C（`src/cycle_timer.c`）で、時計の関数だけを使う。ポートではない。

## 4. ポートではないもの

`mailbox`、`ipc_flags`、`spsc_ring` は、`<stdatomic.h>` を使う純粋C11で `src/` に実装する（ステップ④）。
SDKに触らないので、シミュレータは本物をそのままリンクし、コア1の役（パケットを書く、旗を立てる、ログを読む）を自分で演じる。

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

## 6. `control_loop_run_once(c, ev)` の方針（ステップ③b）

```c
void control_loop_run_once(control_loop_t *c, const imu_int1_event_t *ev);
```

- 中では待たない。INT1の記録 `ev` は呼ぶ側が渡す。
- 実機: `control_loop_run` が `imu_wait_int1` の結果を渡して呼ぶ。タイムアウト処理もこちら。
- シミュレータ: 仮想時間で作った `ev` を渡して直接呼ぶ。
- 中から呼ぶのは `imu_read_fifo`、`motor_pwm_*`、`battery_update`、`fc_clock_*`、`fc_wdt_feed`、mailbox / ipc_flags だけ。すべてヘッダ越し。
