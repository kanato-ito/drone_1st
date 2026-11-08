"""機体モデルの「差し込み口」。

★ 機体モデル本体（力の式と剛体の運動）は kanato さんが書く ★
このファイルにあるのは、シミュレータ（runner.py）が機体モデルに求める形と、つなぎ込みの確認用の
「試験台モデル」（BenchModel：機体が台に固定されていて、姿勢だけを外から決める）だけ。
BenchModel は物理を計算しない。機体モデルができたら、それに差し替える。

座標と単位の約束（ファームに合わせる）
- 機体座標の向きは kanato さんが決める（設計メモ 6章／ミキサーの TODO）。
  runner.py は imu_mount（機体座標→IMU基板座標の 3×3 行列）で、機体座標の値をIMU基板の軸に直してから
  ファームに渡す。ファームはいまIMU基板の軸のまま計算している（取り付け補正は未実装）。
- 角速度 rad/s、加速度 g（重力加速度を1とする）、デューティ 0..1、電圧 V、時間 s。
- 加速度センサーが出すのは「比力」＝（機体の加速度 − 重力加速度）を機体座標で表したもの。
  水平に静止していれば、上向きの軸に +1g が出る。自由落下なら 0。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np


@dataclass
class Disturbance:
    """外から加える力とトルク（外乱）。シナリオが時刻ごとに決めて、機体モデルの step に渡す。"""

    force_world_n: np.ndarray = field(default_factory=lambda: np.zeros(3))   # 世界座標の力 [N]
    torque_body_nm: np.ndarray = field(default_factory=lambda: np.zeros(3))  # 機体座標のトルク [N·m]


NO_DISTURBANCE = Disturbance()


class DroneModel(Protocol):
    """runner.py が呼ぶ形。kanato さんの機体モデルはこの5つを持てばよい（クラスの継承は不要）。"""

    def reset(self, rng: np.random.Generator) -> None:
        """初期状態に戻す（床の上で水平に静止、など）。ノイズ用の乱数は rng から取る。"""

    def step(self, duty: np.ndarray, dt: float, disturbance: Disturbance) -> None:
        """デューティ（M1..M4）を dt 秒のあいだ加え続けて、状態を1刻み進める。"""

    def imu(self) -> tuple[np.ndarray, np.ndarray]:
        """いまの状態でIMUが出す値（機体座標）。(角速度 [rad/s], 比力 [g])。ノイズ・バイアスもここで足す。"""

    def battery_voltage(self) -> float:
        """電池の端子電圧 [V]（内部抵抗による電圧降下を含む）。ファームのADCはこれを読む。"""

    def log(self) -> dict[str, float | np.ndarray]:
        """グラフに出したい本当の値（本当の姿勢・角速度・各モーターの推力・電流など）。何を返してもよい。"""


# ---------------------------------------------------------------------------
# つなぎ込みの確認用（物理なし）
# ---------------------------------------------------------------------------

def euler_zyx_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Z-Y-X オイラー角（ファームと同じ）→ 機体座標から世界座標への回転行列。"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


class BenchModel:
    """試験台に固定した機体（物理なし）。姿勢は attitude_of_t(t) で外から決める。

    デューティは記録するだけで、機体は動かない。ファームの control_bench と同じ確認
    （傾けたら推定が追従するか、PIDが戻す向きの指令を出すか）をPCでするためのもの。
    世界座標は z 上向き（重力は −z）として比力を作る。
    """

    def __init__(self, attitude_of_t: Callable[[float], tuple[float, float, float]] | None = None,
                 battery_v: float = 3.8, gyro_bias_rps=(0.0, 0.0, 0.0), gyro_noise_rps: float = 0.0,
                 accel_noise_g: float = 0.0):
        self._att = attitude_of_t or (lambda t: (0.0, 0.0, 0.0))
        self._batt = float(battery_v)
        self._bias = np.asarray(gyro_bias_rps, dtype=np.float64)
        self._gn = float(gyro_noise_rps)
        self._an = float(accel_noise_g)
        self.reset(np.random.default_rng(0))

    def reset(self, rng: np.random.Generator) -> None:
        self._rng = rng
        self.t = 0.0
        self._r = euler_zyx_to_matrix(*self._att(0.0))
        self._omega = np.zeros(3)
        self.duty = np.zeros(4)

    def step(self, duty: np.ndarray, dt: float, disturbance: Disturbance) -> None:
        self.duty = np.array(duty, dtype=np.float64)
        self.t += dt
        r_new = euler_zyx_to_matrix(*self._att(self.t))
        # 機体座標の角速度: R_old^T R_new ≈ I + [ω]× dt
        rel = self._r.T @ r_new
        skew = 0.5 * (rel - rel.T)
        self._omega = np.array([skew[2, 1], skew[0, 2], skew[1, 0]]) / dt
        self._r = r_new

    def imu(self) -> tuple[np.ndarray, np.ndarray]:
        gyro = self._omega + self._bias
        accel = self._r.T @ np.array([0.0, 0.0, 1.0])   # 静止: 比力 = −重力 = 世界の上向きに 1g
        if self._gn > 0.0:
            gyro = gyro + self._rng.normal(0.0, self._gn, 3)
        if self._an > 0.0:
            accel = accel + self._rng.normal(0.0, self._an, 3)
        return gyro, accel

    def battery_voltage(self) -> float:
        return self._batt

    def log(self) -> dict[str, float | np.ndarray]:
        roll, pitch, yaw = self._att(self.t)
        return {"true_angle_rad": np.array([roll, pitch, yaw]), "true_rate_rps": self._omega.copy()}
