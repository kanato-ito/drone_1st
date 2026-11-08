# sim — 実機前の確認用シミュレータ

ファームの制御のコード（`firmware/src` の純粋Cの部分）をそのままPCでビルドし、機体モデルにつないで、
プロペラを回す前に符号の誤り・外乱・積分ワインドアップ・ゲインの当たりを確かめる。
同じ機体モデルを、あとで強化学習にも使う。決まったことは設計メモ 6章「シミュレータ」。

## 分担

| 部分 | 書く人 | 場所 |
| --- | --- | --- |
| 機体モデル（力の式・剛体の回転と並進・床・IMUのノイズ・電池の内部抵抗） | kanato | 新しいファイル（例 `fcsim/drone.py`）。形は `fcsim/model.py` の `DroneModel` |
| ポートの偽物、1周期の呼び出し、ctypes、時間の進め方、グラフ、試験 | Claude | `c/`、`fcsim/fc.py` `runner.py` `plot.py`、`tests/` |
| MuJoCo版（離陸・着地） | あとで | — |

## 使い方

```sh
# 1回だけ
pip install -r sim/requirements.txt

# ファームのC（純粋な部分）とポートの偽物を共有ライブラリにする。ファームを変えたらもう一度
cmake -S sim -B sim/build -DCMAKE_BUILD_TYPE=Release
cmake --build sim/build

# 試験（sim ディレクトリで）
cd sim
python -m pytest -q tests
python -m fcsim.scenarios.bench_tilt      # グラフが sim/out/bench_tilt.png にできる
```

Mac では `sim/build/libfcsim.dylib`、Linux では `libfcsim.so` ができる。別の場所のものを使うときは環境変数 `FCSIM_LIB`。

## しくみ

```
  機体モデル（kanato）            runner.py                       libfcsim（C）
  ┌──────────────┐  step(duty)   ┌───────────────────┐  push_imu   ┌──────────────────────────┐
  │ 力の式・剛体  │◀──────────────│ 物理の1刻み=1/7519秒 │────────────▶│ port_sim.c（IMU・PWM・    │
  │ IMU・電池     │──────────────▶│ 8サンプルごとに      │  step       │   時計・電池ADCの偽物）   │
  └──────────────┘  imu(), 電圧   │ ファームの1周期      │◀────────────│ fcsim.c（1周期）          │
                                  └───────────────────┘  duty       │ firmware/src/*.c（そのまま）│
                                                                    └──────────────────────────┘
```

- **ファームとの境目**は `docs/hw_boundary.md` のポート4つ（IMU・PWM・時計/WDT・電池ADC）。
  `c/port_sim.c` が同じヘッダを実装し、`firmware/cmake/fc_sources.cmake` の `FC_CORE_SOURCES` をそのままリンクする。
- **1周期**（`c/fcsim.c` の `fcsim_step`）は★仮★。いまは `firmware/apps/control_bench.c` と同じ順に
  ③aの流れ（姿勢推定 → 2段PID → ミキサー → 推力換算 → PWM）を呼び、アームも control_bench と同じ簡易手順。
  ステップ③bでファームに `control_loop_run_once(c, ev)` が入ったら、中身をそれに置き換える
  （アーム要求・目標はメールボックスへ。Python側の呼び方は変えない）。
- **時間**: 物理の1刻み = IMUの1サンプル（1/7519秒）。8サンプルごとにファームの1周期（約940Hz）。
  デューティは次に変わるまで同じ値を加え続ける。`Simulation(pwm_delay_samples=n)` で処理時間ぶん遅らせられる。
- **座標**: 機体モデルは機体の軸（x前・y左・z上）で値を出す。`Simulation(imu_mount=...)` の回転行列で
  IMU基板の軸（x右・y前・z上）に直してからファームに渡す。ファームはいま基板の軸のまま計算している
  （`params.h` の `axis_map` は未実装）。
- ファームの状態は静的変数1組なので、1プロセスで動かせる機体は1機。強化学習で並列にするときはプロセスを分ける。

## 機体モデルの形（`fcsim/model.py` の `DroneModel`）

| メソッド | すること |
| --- | --- |
| `reset(rng)` | 初期状態に戻す。ノイズの乱数は `rng` から取る |
| `step(duty, dt, disturbance)` | デューティ（M1..M4、0..1）を `dt` 秒加え続けて1刻み進める。`disturbance` は外乱（世界座標の力 [N]、機体座標のトルク [N·m]） |
| `imu()` | （角速度 [rad/s], 比力 [g]）を機体の軸で返す。比力 = 機体の加速度 − 重力加速度（静止なら上向きに +1g） |
| `battery_voltage()` | 端子電圧 [V]（内部抵抗の電圧降下を含む） |
| `log()` | グラフに出したい本当の値（`true_angle_rad` を返すと姿勢のグラフに点線で重なる） |

`BenchModel` は物理なしの試験台（姿勢を外から決めるだけ）。つなぎ込みの確認用で、機体モデルができたら差し替える。

## 試験（`tests/test_glue.py`）

つなぎ込みが正しいことだけを見る（物理は見ない）: 刻みがファームと同じ、1周期でFIFOを全部読む、
静止1秒・水平でアーム、傾けたら推定が追従し戻す向きの指令がミキサーの符号表どおりに配られる、
ディスアームで0、電池電圧がADCを通って届く、`imu_mount` で軸が回る、PWMの遅れ、パラメータの検査。
