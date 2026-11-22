"""時間の進め方（設計メモ 6章「シミュレータ」）。

- 物理の1刻み = IMUの1サンプル（1/7519 秒。ファームの imu_sample_period_s と同じ値）。
- 刻みごとに: 機体モデルを進める → IMUの値をIMU基板の軸に直してファームのFIFOに積む → 電池電圧をADCに渡す。
- 8サンプルたまるたびに（ウォーターマーク16ワード＝8組）、時計を合わせてファームの1周期を呼ぶ。
- ファームが決めたデューティは、次に変わるまで同じ値を機体モデルに加え続ける（PWMと同じ）。
  pwm_delay_samples で「INT1からPWM更新まで」の処理時間ぶん遅らせられる（既定0）。
- 操縦はPCの通信係と同じく packet_period_s（既定 20ms = 50Hz）ごとに操縦パケットを送る（コア1の役は C 側）。
  Controls.arm は「飛ばしたい」という操縦者の意思。runner が操縦者の手順を演じる:
    飛行指示待機の間は、スロットル0でアームのボタンを 0.1秒押す・0.1秒離す、をくり返す（ファームは0→1で判定）。
    飛行中に arm=False になったらディスアームの旗を送る。飛行・非常時降下以外ではスロットルは0で送る。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .fc import (FLAG_ARM_REQ, FLAG_DISARM, FLAG_ESTOP, FLAG_TAKEOVER_PID, FS_EMERGENCY_DESCENT, FS_FLIGHT,
                 FlightComputer)
from .model import NO_DISTURBANCE, Disturbance, DroneModel

SAMPLES_PER_CYCLE = 8   # FIFOのウォーターマーク 16ワード ÷ 2（ジャイロ＋加速度で1組）

# 機体の軸（x前・y左・z上）→ IMU基板の軸（x右・y前・z上）。設計メモの axis_map = {2, -1, 3} の逆向き。
#   基板x（右）= −機体y、基板y（前）= +機体x、基板z = +機体z
# ファームは imu_mount_apply で基板→機体に戻すので、ここと axis_map がそろっていれば機体の軸で一致する。
MOUNT_BODY_TO_BOARD = np.array([[0.0, -1.0, 0.0],
                                [1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0]])


@dataclass
class Controls:
    """操縦者の入力。角度は rad、推力は1モーターあたりの g（runner がスロットル 0..1 に直して送る）。"""

    arm: bool = False         # 飛ばしたい（アームの手順は runner が演じる。上の説明）
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0      # ファームはアーム時の値との差で目標を作る
    thrust_g: float = 0.0
    estop: bool = False       # 非常停止の旗
    takeover_pid: bool = False


@dataclass
class Scenario:
    duration_s: float
    controls: Callable[[float], Controls]
    disturbance: Callable[[float], Disturbance] = field(default=lambda t: NO_DISTURBANCE)
    battery_override_v: Callable[[float], float] | None = None   # 機体モデルの電圧の代わりに使う（試験用）
    fc_setup: Callable[[FlightComputer], None] | None = None
    # ↑ ファームの起動処理（params は既定値に戻る）のあとに呼ぶ。ゲインや params を変えて比べるとき用
    link_up: Callable[[float], bool] = field(default=lambda t: True)
    # ↑ False の間は操縦パケットを送らない（通信途絶の試験用）
    int1_ok: Callable[[float], bool] = field(default=lambda t: True)
    # ↑ False の間は INT1 が来ない（IMU途絶の試験用。サンプルはFIFOにたまり続ける）


class Log(dict):
    """1周期ごとの記録。キー → numpy配列（行 = 周期）。't' は周期の時刻 [s]。"""


class Simulation:
    def __init__(self, model: DroneModel, fc: FlightComputer | None = None,
                 imu_mount: np.ndarray | None = None, pwm_delay_samples: int = 0, seed: int = 0,
                 packet_period_s: float = 0.02, arm_press_s: float = 0.1):
        self.model = model
        self.fc = fc or FlightComputer()
        # 機体座標 → IMU基板座標（既定: 設計メモの取り付け）。偽IMUは基板の軸で出す（並べ替えはファームの仕事）
        self.imu_mount = MOUNT_BODY_TO_BOARD if imu_mount is None else np.asarray(imu_mount, dtype=np.float64)
        if not np.allclose(self.imu_mount @ self.imu_mount.T, np.eye(3), atol=1e-9) or \
                not np.isclose(np.linalg.det(self.imu_mount), 1.0):
            raise ValueError("imu_mount は回転行列（直交・行列式+1）にすること。左手系は不可")
        self.pwm_delay_samples = int(pwm_delay_samples)
        self.seed = seed
        self.packet_period_s = float(packet_period_s)
        self.arm_press_s = float(arm_press_s)

    def _packet(self, c: Controls, t: float, state: int, full_thrust_g: float) -> tuple[float, int]:
        """操縦者と通信係の手順: (スロットル 0..1, 旗)。"""
        flying = state in (FS_FLIGHT, FS_EMERGENCY_DESCENT)
        flags = 0
        if c.arm:
            if int(t / self.arm_press_s) % 2 == 0:
                flags |= FLAG_ARM_REQ          # 押す・離すをくり返す（待機に着く前の押しっぱなしではアームしない）
        elif state == FS_FLIGHT:
            flags |= FLAG_DISARM
        if c.estop:
            flags |= FLAG_ESTOP
        if c.takeover_pid:
            flags |= FLAG_TAKEOVER_PID
        throttle = float(np.clip(c.thrust_g / full_thrust_g, 0.0, 1.0)) if (flying and c.arm) else 0.0
        return throttle, flags

    def run(self, scenario: Scenario) -> Log:
        fc, model = self.fc, self.model
        model.reset(np.random.default_rng(self.seed))
        v0 = scenario.battery_override_v(0.0) if scenario.battery_override_v else model.battery_voltage()
        fc.reset(battery_v=v0)
        if scenario.fc_setup is not None:
            scenario.fc_setup(fc)
        full_thrust_g = fc.param("airframe.throttle_full_thrust_g")
        dt = fc.sample_period_s
        n_steps = int(round(scenario.duration_s / dt))

        duty = np.zeros(4)
        pending: list[tuple[int, np.ndarray]] = []   # (適用するステップ, デューティ)
        rows: list[dict] = []
        t = 0.0
        next_packet_t = 0.0
        state = fc.state
        c = Controls()

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
                fc.set_time_us(int(round(t * 1e6)))
                if t + 1e-9 >= next_packet_t:
                    next_packet_t += self.packet_period_s
                    c = scenario.controls(t)
                    if scenario.link_up(t):
                        throttle, flags = self._packet(c, t, state, full_thrust_g)
                        fc.send_packet(c.roll_rad, c.pitch_rad, c.yaw_rad, throttle, flags)
                int1 = scenario.int1_ok(t)
                again = fc.step(int1)
                for _ in range(4):                 # 読み終えてもピンが High: 待たずにすぐ読み直す（実機と同じ）
                    if not again:
                        break
                    again = fc.step(True)
                new_duty = fc.duty()
                if self.pwm_delay_samples > 0:
                    pending.append((k + self.pwm_delay_samples, new_duty))
                else:
                    duty = new_duty
                tel = fc.telemetry()
                state = int(tel.state)
                row = tel.as_dict()
                row["t"] = t
                row["battery_v_true"] = v
                row["cmd_thrust_g"] = c.thrust_g
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
