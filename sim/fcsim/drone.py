"""初代Cの機体モデル（kanato が書く）。

runner.py から見ると、Drone クラスが model.py の DroneModel の形（reset / step / imu / battery_voltage / log）を持つ。
中身は5つの部品に分け、手順の 1〜5 の順に1つずつ埋めていく。

  1. Motors     … デューティ → 一次遅れ → 推力・反トルク・電流
  2. Battery    … 合計電流 → 端子電圧（内部抵抗の電圧降下）
  3. RigidBody  … 力とトルク → 角速度・姿勢（回転）
  4. RigidBody  … 力 → 速度・位置（並進）と床
  5. Imu        … 本当の状態 → 角速度と比力（ノイズ・バイアス）

約束（model.py の冒頭と同じ）
- 機体座標: x前・y左・z上。世界座標: z上（重力は −z）。
- モーター: 添字 0..3 = M1..M4。M1 右後・時計回り、M2 右前・反時計回り、M3 左後・反時計回り、M4 左前・時計回り（上から見て）。
- 物理の式は SI 単位（N, N·m, kg, m, s）。推力の実測式だけ g で与えられているので、Motors の中で N に直す。
- IMU が返すのは角速度 [rad/s] と比力 [g]（機体座標）。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .model import Disturbance

G0 = 9.80665            # 重力加速度 [m/s²]
GRAM_TO_N = G0 * 1e-3   # 推力 [g] → [N]


# ---------------------------------------------------------------------------
# パラメータ（値はここに集める。TODO は出典を確かめて埋める）
# ---------------------------------------------------------------------------

@dataclass
class MotorParams:
    k_thrust_g_per_v: float = 9.4    # 推力[g] ≒ k × デューティ × 電圧[V]（HW設計書7章、実測）
    tau_s: float = 0.070             # 一次遅れの時定数 [s]（実測）
    k_q_m: float = 0.0               # TODO: 反トルク = k_q × 推力[N] の係数 [m]（見積もり→あとで実測）
    k_i_a_per_g: float = 0.0         # TODO: 電流と推力の関係（HW設計書6章の実測の行から）
    spin: np.ndarray = field(default_factory=lambda: np.array([+1.0, -1.0, -1.0, +1.0]))
    # ↑ 反トルクの向き（機体z軸まわりの＋が+1）。ミキサーの符号表のヨーの列と同じ


@dataclass
class BatteryParams:
    v_open_v: float = 3.8            # TODO: 開放電圧 [V]（満充電 4.35V LiHV〜下限。試験ごとに変える）
    r_internal_ohm: float = 0.0      # TODO: 内部抵抗 [Ω]（未測定。値を振って試す）


@dataclass
class BodyParams:
    mass_kg: float = 0.050           # TODO: 実測（HW設計書6章の目標 約50g）
    inertia_kgm2: np.ndarray = field(default_factory=lambda: np.zeros(3))   # TODO: Ixx, Iyy, Izz（見積もり）
    motor_pos_m: np.ndarray = field(default_factory=lambda: np.zeros((4, 3)))
    # ↑ TODO: 各モーターの位置（機体座標、重心から）。HW設計書5章の対角の軸間距離から


@dataclass
class ImuParams:
    gyro_noise_rps: float = 0.0      # TODO: 1サンプルあたりの雑音（標準偏差）
    accel_noise_g: float = 0.0       # TODO
    gyro_bias_rps: np.ndarray = field(default_factory=lambda: np.zeros(3))   # TODO: 例 z軸 +0.44dps（HW設計書7章）


@dataclass
class DroneParams:
    motor: MotorParams = field(default_factory=MotorParams)
    battery: BatteryParams = field(default_factory=BatteryParams)
    body: BodyParams = field(default_factory=BodyParams)
    imu: ImuParams = field(default_factory=ImuParams)


# ---------------------------------------------------------------------------
# 1. モーター
# ---------------------------------------------------------------------------

class Motors:
    """4つのモーター。状態は今の推力 [g]（長さ4）。"""

    def __init__(self, p: MotorParams):
        self.p = p
        self.thrust_g = np.zeros(4)

    def reset(self) -> None:
        self.thrust_g = np.zeros(4)

    def step(self, duty: np.ndarray, v_batt: float, dt: float) -> None:
        """デューティ（0..1）と端子電圧 [V] で、推力を dt 秒ぶん進める。"""
        raise NotImplementedError("手順1: 目指す推力 → 一次遅れ")

    def thrust_n(self) -> np.ndarray:
        """各モーターの推力 [N]（長さ4）。"""
        raise NotImplementedError("手順1: g → N")

    def reaction_torque_nm(self) -> np.ndarray:
        """各モーターの反トルク [N·m]（機体z軸まわり、長さ4。向きは spin）。"""
        raise NotImplementedError("手順1: 反トルク")

    def current_a(self) -> np.ndarray:
        """各モーターの電流 [A]（長さ4）。"""
        raise NotImplementedError("手順1: 電流")


# ---------------------------------------------------------------------------
# 2. 電池
# ---------------------------------------------------------------------------

class Battery:
    def __init__(self, p: BatteryParams):
        self.p = p
        self.v_terminal = p.v_open_v

    def reset(self) -> None:
        self.v_terminal = self.p.v_open_v

    def update(self, total_current_a: float) -> None:
        """合計電流 [A] から端子電圧を決める。"""
        raise NotImplementedError("手順2: 内部抵抗の電圧降下")


# ---------------------------------------------------------------------------
# 3・4. 剛体（回転・並進・床）
# ---------------------------------------------------------------------------

class RigidBody:
    """状態: 位置・速度（世界座標）、姿勢（クォータニオン、機体→世界）、角速度（機体座標）。"""

    def __init__(self, p: BodyParams):
        self.p = p
        self.reset()

    def reset(self) -> None:
        self.pos_m = np.zeros(3)                     # 世界座標
        self.vel_mps = np.zeros(3)                   # 世界座標
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])   # (w, x, y, z)。水平・北向き
        self.omega_rps = np.zeros(3)                 # 機体座標
        self.accel_mps2 = np.zeros(3)                # 世界座標の加速度（IMUの比力に使う）

    def rotation(self) -> np.ndarray:
        """機体座標 → 世界座標の回転行列（3×3）。"""
        raise NotImplementedError("手順3: クォータニオン → 回転行列")

    def step(self, force_body_n: np.ndarray, torque_body_nm: np.ndarray, dt: float,
             disturbance: Disturbance) -> None:
        """機体座標の力・トルク（モーターから）と外乱で、dt 秒ぶん進める。"""
        raise NotImplementedError("手順3: 回転 / 手順4: 並進と床")

    def euler_zyx(self) -> np.ndarray:
        """ロール・ピッチ・ヨー [rad]（Z-Y-X、ファームと同じ定義）。グラフの「本当」の線に使う。"""
        raise NotImplementedError("手順3: 姿勢 → オイラー角")


# ---------------------------------------------------------------------------
# 5. IMU
# ---------------------------------------------------------------------------

class Imu:
    def __init__(self, p: ImuParams):
        self.p = p
        self.rng = np.random.default_rng(0)

    def reset(self, rng: np.random.Generator) -> None:
        self.rng = rng

    def read(self, body: RigidBody) -> tuple[np.ndarray, np.ndarray]:
        """(角速度 [rad/s], 比力 [g])、機体座標。"""
        raise NotImplementedError("手順5: 比力 = 加速度 − 重力、ノイズとバイアス")


# ---------------------------------------------------------------------------
# 組み立て（runner.py が呼ぶ形）
# ---------------------------------------------------------------------------

class Drone:
    """DroneModel の形。部品を決まった順に呼ぶ。"""

    def __init__(self, params: DroneParams | None = None):
        self.params = params or DroneParams()
        self.motors = Motors(self.params.motor)
        self.battery = Battery(self.params.battery)
        self.body = RigidBody(self.params.body)
        self.imu_sensor = Imu(self.params.imu)

    def reset(self, rng: np.random.Generator) -> None:
        self.motors.reset()
        self.battery.reset()
        self.body.reset()
        self.imu_sensor.reset(rng)

    def step(self, duty: np.ndarray, dt: float, disturbance: Disturbance) -> None:
        # TODO: 部品をつなぐ。順番の案:
        #   1. motors.step(duty, 前の刻みの端子電圧, dt)
        #   2. battery.update(4つの電流の合計)
        #   3. モーターの推力・反トルクを、機体座標の合計の力とトルクにまとめる
        #   4. body.step(力, トルク, dt, disturbance)
        raise NotImplementedError("手順1〜4がそろったら")

    def imu(self) -> tuple[np.ndarray, np.ndarray]:
        return self.imu_sensor.read(self.body)

    def battery_voltage(self) -> float:
        return float(self.battery.v_terminal)

    def log(self) -> dict[str, float | np.ndarray]:
        return {
            "true_angle_rad": self.body.euler_zyx(),
            "true_rate_rps": self.body.omega_rps.copy(),
            "pos_m": self.body.pos_m.copy(),
            "thrust_g": self.motors.thrust_g.copy(),
        }
