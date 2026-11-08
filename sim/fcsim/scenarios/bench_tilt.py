"""つなぎ込みの確認: 試験台に固定した機体を傾けて、ファームが戻す向きの指令を出すかを見る。

機体は動かない（BenchModel、物理なし）。firmware/apps/control_bench.c の合格条件 1〜3 をPCで見るのと同じ。
  0〜2.5秒   水平で静止 → 静止が1秒続くとバイアスが求まり、アームする
  3〜3.3秒   ロールを +10° まで傾ける（0.3秒かけて）
  5〜5.3秒   水平に戻す
  6〜6.3秒   ピッチを +10° まで傾ける
  8〜8.3秒   水平に戻す
グラフ: sim/out/bench_tilt.png

    cd sim && python -m fcsim.scenarios.bench_tilt
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from fcsim import BenchModel, Controls, Scenario, Simulation
from fcsim.plot import plot_log

DEG = np.pi / 180.0
HOVER_THRUST_G = 12.5   # HW設計書6章: 50g ÷ 4


def ramp(t: float, t0: float, t1: float) -> float:
    """t0〜t1 で 0→1 に滑らかに変わる（ジャイロに無限大の角速度を出さないため）。"""
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    x = (t - t0) / (t1 - t0)
    return x * x * (3.0 - 2.0 * x)


def bench_attitude(t: float) -> tuple[float, float, float]:
    roll = 10.0 * DEG * (ramp(t, 3.0, 3.3) - ramp(t, 5.0, 5.3))
    pitch = 10.0 * DEG * (ramp(t, 6.0, 6.3) - ramp(t, 8.0, 8.3))
    return roll, pitch, 0.0


def scenario() -> Scenario:
    return Scenario(duration_s=9.5,
                    controls=lambda t: Controls(arm=True, thrust_g=HOVER_THRUST_G))


def main() -> None:
    model = BenchModel(bench_attitude, battery_v=3.8, gyro_noise_rps=0.002, accel_noise_g=0.002)
    log = Simulation(model).run(scenario())
    out = Path(__file__).resolve().parents[2] / "out" / "bench_tilt.png"
    plot_log(log, out, title="試験台で傾ける（物理なし）")

    armed = log["armed"] > 0.5
    t_arm = log["t"][armed][0] if armed.any() else float("nan")
    i4 = np.searchsorted(log["t"], 4.5)
    print(f"アーム: {t_arm:.2f} s")
    print(f"t=4.5s（ロール+10°）: 推定 roll={log['angle_rad'][i4, 0] / DEG:+.2f}°  "
          f"ロール指令={log['out_torque_g'][i4, 0]:+.2f} g  duty={np.round(log['duty'][i4], 3)}")
    print(f"グラフ: {out}")


if __name__ == "__main__":
    main()
