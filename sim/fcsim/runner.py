"""時間の進め方（設計メモ 6章「シミュレータ」）。

- 物理の1刻み = IMUの1サンプル（1/7519 秒。ファームの imu_sample_period_s と同じ値）。
- 刻みごとに: 機体モデルを進める → IMUの値をIMU基板の軸に直してファームのFIFOに積む → 電池電圧をADCに渡す。
- 8サンプルたまるたびに（ウォーターマーク16ワード＝8組）、時計を合わせてファームの1周期を呼ぶ。
- ファームが決めたデューティは、次に変わるまで同じ値を機体モデルに加え続ける（PWMと同じ）。
  pwm_delay_samples で「INT1からPWM更新まで」の処理時間ぶん遅らせられる（既定0）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .fc import FlightComputer
from .model import NO_DISTURBANCE, Disturbance, DroneModel

SAMPLES_PER_CYCLE = 8   # FIFOのウォーターマーク 16ワード ÷ 2（ジャイロ＋加速度で1組）


@dataclass
class Controls:
    """操縦（メールボックスに入る中身の代わり）。角度は rad、推力は1モーターあたりの g。"""

    arm: bool = False
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0      # アーム時の向きからのずれ
    thrust_g: float = 0.0


@dataclass
class Scenario:
    duration_s: float
    controls: Callable[[float], Controls]
    disturbance: Callable[[float], Disturbance] = field(default=lambda t: NO_DISTURBANCE)
    battery_override_v: Callable[[float], float] | None = None   # 機体モデルの電圧の代わりに使う（試験用）


class Log(dict):
    """1周期ごとの記録。キー → numpy配列（行 = 周期）。't' は周期の時刻 [s]。"""


class Simulation:
    def __init__(self, model: DroneModel, fc: FlightComputer | None = None,
                 imu_mount: np.ndarray | None = None, pwm_delay_samples: int = 0, seed: int = 0):
        self.model = model
        self.fc = fc or FlightComputer()
        # 機体座標 → IMU基板座標。kanato さんが決めた取り付けに合わせて変える（既定: 同じ向き）
        self.imu_mount = np.eye(3) if imu_mount is None else np.asarray(imu_mount, dtype=np.float64)
        if not np.allclose(self.imu_mount @ self.imu_mount.T, np.eye(3), atol=1e-9) or \
                not np.isclose(np.linalg.det(self.imu_mount), 1.0):
            raise ValueError("imu_mount は回転行列（直交・行列式+1）にすること。左手系は不可")
        self.pwm_delay_samples = int(pwm_delay_samples)
        self.seed = seed

    def run(self, scenario: Scenario) -> Log:
        fc, model = self.fc, self.model
        fc.reset()
        model.reset(np.random.default_rng(self.seed))
        dt = fc.sample_period_s
        n_steps = int(round(scenario.duration_s / dt))

        duty = np.zeros(4)
        pending: list[tuple[int, np.ndarray]] = []   # (適用するステップ, デューティ)
        rows: list[dict] = []
        t = 0.0

        for k in range(1, n_steps + 1):
            # ---- 物理を1刻み ----
            model.step(duty, dt, scenario.disturbance(t))
            t = k * dt

            gyro_b, accel_b = model.imu()
            fc.push_imu(self.imu_mount @ gyro_b, self.imu_mount @ accel_b)
            v = scenario.battery_override_v(t) if scenario.battery_override_v else model.battery_voltage()
            fc.set_battery_voltage(v)

            # ---- 8サンプルごとにファームの1周期 ----
            if k % SAMPLES_PER_CYCLE == 0:
                c = scenario.controls(t)
                fc.set_arm_request(c.arm)
                fc.set_target(c.roll_rad, c.pitch_rad, c.yaw_rad, c.thrust_g)
                t_us = int(round(t * 1e6))
                fc.set_time_us(t_us)
                fc.step(t_us)
                new_duty = fc.duty()
                if self.pwm_delay_samples > 0:
                    pending.append((k + self.pwm_delay_samples, new_duty))
                else:
                    duty = new_duty
                row = fc.telemetry().as_dict()
                row["t"] = t
                row["battery_v_true"] = v
                for key, val in model.log().items():
                    row["model_" + key] = val
                rows.append(row)

            while pending and pending[0][0] <= k:
                duty = pending.pop(0)[1]

        return _to_log(rows)


def _to_log(rows: list[dict]) -> Log:
    log = Log()
    if not rows:
        return log
    for key in rows[0]:
        log[key] = np.array([r[key] for r in rows])
    return log
