"""初代Cの機体モデル（空中＋高さ0で止まるだけの床。自作版）。

runner.py から見ると、Drone クラスが model.py の DroneModel の形（reset / step / imu / battery_voltage / log）を持つ。
中身は5つの部品に分け、1刻みごとに 1→2→3・4 の順に呼ぶ（5 は runner が imu() で読む）。

  1. Motors     … デューティ → 一次遅れ → 推力・反トルク・電流
  2. Battery    … 合計電流 → 端子電圧（内部抵抗の電圧降下）
  3. RigidBody  … 力とトルク → 角速度・姿勢（回転、オイラーの運動方程式とクォータニオン）
  4. RigidBody  … 力 → 速度・位置（並進）と床
  5. Imu        … 本当の状態 → 角速度と比力（ノイズ・バイアス）

約束（model.py の冒頭と同じ）
- 機体座標: x前・y左・z上。世界座標: z上（重力は −z）。
- モーター: 添字 0..3 = M1..M4。M1 右後・時計回り、M2 右前・反時計回り、M3 左後・反時計回り、M4 左前・時計回り（上から見て）。
- 物理の式は SI 単位（N, N·m, kg, m, s）。推力の実測式だけ g で与えられているので、Motors の中で N に直す。
- IMU が返すのは角速度 [rad/s] と比力 [g]（機体座標）。

値の出典は各パラメータの横に書いた。「見積もり」は実測で置き換える値、「仮」は値を振って試す値。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .model import Disturbance

G0 = 9.80665            # 重力加速度 [m/s²]
GRAM_TO_N = G0 * 1e-3   # 推力 [g] → [N]
DEG = np.pi / 180.0

# 基板の外形（/mnt の kicad/layout/outline_square62_points.md）: モーター穴の中心は Pico の中心から
# 前後・左右に 42.43mm（対角の軸間距離 120mm）。重心は Pico の中心にあるとみなす（電池の位置で変わる）。
ARM_XY_M = 0.04243


# ---------------------------------------------------------------------------
# パラメータ（値はここに集める）
# ---------------------------------------------------------------------------

@dataclass
class MotorParams:
    k_thrust_g_per_v: float = 9.4    # 推力[g] ≒ k × デューティ × 電圧[V]（HW設計書7章、M1の実測）
    tau_s: float = 0.070             # 一次遅れの時定数 [s]（実測）
    k_q_m: float = 0.005             # 反トルク = k_q × 推力[N] の係数 [m]。見積もり（0.003〜0.02）→ヨーの試験で実測
    k_i_a_per_g: float = 0.089       # 電流[A] ≒ k × 推力[g]（HW設計書6章の全開3点と7章の70% 23.8g・2.14A）
    spin: np.ndarray = field(default_factory=lambda: np.array([+1.0, -1.0, -1.0, +1.0]))
    # ↑ 反トルクの向き（機体z軸まわりの＋が+1）。時計回りのプロペラは機体を反時計回り（+z）に回す。
    #   ミキサーの符号表のヨーの列と同じ
    thrust_scale: np.ndarray = field(default_factory=lambda: np.ones(4))
    # ↑ モーターの個体差（1.0 = M1 と同じ）。9.4 は M1 だけの値なので、試験でばらつかせる


@dataclass
class BatteryParams:
    v_open_v: float = 3.8            # 開放電圧 [V]（満充電 4.35V LiHV〜下限。試験ごとに変える）
    r_internal_ohm: float = 0.0      # 内部抵抗 [Ω]（未測定。電池を買ったら測る。試験では 0〜0.15 を振る）


@dataclass
class BodyParams:
    mass_kg: float = 0.050           # HW設計書6章の概算 47〜52g（電池込み）。部品がそろったら実測
    inertia_kgm2: np.ndarray = field(default_factory=lambda: np.array([5.5e-5, 5.5e-5, 1.0e-4]))
    # ↑ Ixx, Iyy, Izz の見積もり。モーター＋プロペラ・固定具（6g×4）を軸の位置の点、残り（約26g）を
    #   62mm角の板とみなした値: Ixx ≈ 4×0.006×0.0424² + 0.026×0.062²/12 ≈ 4.3e-5 + 0.8e-5、電池の厚みで少し足す。
    #   Izz ≈ Ixx + Iyy（平たい物体）。Crazyflie（27g・対角92mm、Ixx 1.7e-5）を重さと寸法で拡大しても同じくらい
    motor_pos_m: np.ndarray = field(default_factory=lambda: np.array([
        [-ARM_XY_M, -ARM_XY_M, 0.0],   # M1 右後
        [+ARM_XY_M, -ARM_XY_M, 0.0],   # M2 右前
        [-ARM_XY_M, +ARM_XY_M, 0.0],   # M3 左後
        [+ARM_XY_M, +ARM_XY_M, 0.0],   # M4 左前
    ]))
    drag_n_per_mps: float = 0.01     # 並進の空気抵抗 F = −c·v [N/(m/s)]。見積もり（Crazyflie の値の程度）
    angular_drag_nm_per_rps: float = 0.0   # 回転の空気抵抗 τ = −c·ω。既定は無し（効かせて比べる用）


@dataclass
class ImuParams:
    # 1サンプル（7519Hz）ごとの雑音の標準偏差 = 雑音密度 × √(帯域)。帯域は ODR の半分とした。
    # 角速度: HW設計書7章「30Hz以下の3軸RMS 0.034dps」→ 1軸 0.020dps ÷ √30Hz ≈ 3.6 mdps/√Hz
    #         （データシートの雑音密度と同じくらい）→ × √3760Hz ≈ 0.22dps
    gyro_noise_rps: float = 0.22 * DEG
    # 加速度: 測っていない。データシート程度の 100 µg/√Hz と置いた → × √3760Hz ≈ 6mg
    accel_noise_g: float = 0.006
    gyro_bias_rps: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.44 * DEG]))
    # ↑ 機体座標。HW設計書7章: 静止でも z軸が +0.44dps ずれていた（x・yは書かれていないので0）
    accel_bias_g: np.ndarray = field(default_factory=lambda: np.zeros(3))


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
        """デューティ（0..1）と端子電圧 [V] で、推力を dt 秒ぶん進める。

        目指す推力（定常値）は実測式 T_ss = k × duty × V。そこへ時定数 τ の一次遅れで近づく。
        dt の間 T_ss が一定なら、一次遅れの厳密な解は T += (T_ss − T)(1 − e^(−dt/τ))。
        """
        duty = np.clip(np.asarray(duty, dtype=float), 0.0, 1.0)
        t_ss = self.p.k_thrust_g_per_v * self.p.thrust_scale * duty * max(v_batt, 0.0)
        self.thrust_g = self.thrust_g + (t_ss - self.thrust_g) * (1.0 - np.exp(-dt / self.p.tau_s))

    def thrust_n(self) -> np.ndarray:
        """各モーターの推力 [N]（長さ4）。"""
        return self.thrust_g * GRAM_TO_N

    def reaction_torque_nm(self) -> np.ndarray:
        """各モーターの反トルク [N·m]（機体z軸まわり、長さ4。向きは spin）。同じ推力なら4つの和は0。"""
        return self.p.spin * self.p.k_q_m * self.thrust_n()

    def current_a(self) -> np.ndarray:
        """各モーターの電流 [A]（長さ4）。係数は A/g なので推力は g のまま掛ける。"""
        return self.p.k_i_a_per_g * self.thrust_g


# ---------------------------------------------------------------------------
# 2. 電池
# ---------------------------------------------------------------------------

class Battery:
    """端子電圧 = 開放電圧 − 内部抵抗 × 合計電流。

    モーターはこの刻みの始めの（＝前の刻みで決めた）端子電圧で回す。同じ刻みの中で
    「電流 → 電圧 → 推力 → 電流」と回すと式が自分自身を参照するので、1刻み（0.13ms）遅らせて切る。
    モーターの時定数 70ms に比べて十分短いので、結果はほとんど変わらない。
    """

    def __init__(self, p: BatteryParams):
        self.p = p
        self.v_terminal = p.v_open_v
        self.current_a = 0.0

    def reset(self) -> None:
        self.v_terminal = self.p.v_open_v
        self.current_a = 0.0

    def update(self, total_current_a: float) -> None:
        """合計電流 [A] から端子電圧を決める。"""
        self.current_a = float(total_current_a)
        self.v_terminal = self.p.v_open_v - self.p.r_internal_ohm * self.current_a


# ---------------------------------------------------------------------------
# 3・4. 剛体（回転・並進・床）
# ---------------------------------------------------------------------------

def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """クォータニオンの積 a ⊗ b（(w, x, y, z)）。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_from_rotvec(v: np.ndarray) -> np.ndarray:
    """回転ベクトル（向き = 回転軸、長さ = 角度 [rad]）→ クォータニオン。"""
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.array([1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]])
    s = np.sin(0.5 * angle) / angle
    return np.array([np.cos(0.5 * angle), s * v[0], s * v[1], s * v[2]])


def quat_from_euler_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Z-Y-X オイラー角（ファームと同じ）→ クォータニオン（機体→世界）。"""
    cr, sr = np.cos(0.5 * roll), np.sin(0.5 * roll)
    cp, sp = np.cos(0.5 * pitch), np.sin(0.5 * pitch)
    cy, sy = np.cos(0.5 * yaw), np.sin(0.5 * yaw)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


class RigidBody:
    """状態: 位置・速度（世界座標）、姿勢（クォータニオン、機体→世界）、角速度（機体座標）。

    積分は半陰的オイラー法（先に速度・角速度を進め、その新しい値で位置・姿勢を進める）。
    刻みが 1/7519 秒と短いので、これで十分な精度になる（試験でエネルギーと角運動量のずれを確かめる）。

    床: 世界の z = 0。床の上にいる間は、上向きの力が重さを超えるまで動かない（位置・速度・角速度を0に保つ）。
    傾いたまま着地する・はねる・すべるといった接触は扱わない（MuJoCo版で見る）。
    """

    def __init__(self, p: BodyParams):
        self.p = p
        self.reset()

    def reset(self, initial_euler_rad: np.ndarray | None = None) -> None:
        self.pos_m = np.zeros(3)                     # 世界座標
        self.vel_mps = np.zeros(3)                   # 世界座標
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])   # (w, x, y, z)。水平・北向き
        if initial_euler_rad is not None:
            self.quat = quat_from_euler_zyx(*initial_euler_rad)
        self.omega_rps = np.zeros(3)                 # 機体座標
        self.accel_mps2 = np.zeros(3)                # 世界座標の加速度（IMUの比力に使う）
        self.on_ground = True

    def rotation(self) -> np.ndarray:
        """機体座標 → 世界座標の回転行列（3×3）。"""
        w, x, y, z = self.quat
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])

    def step(self, force_body_n: np.ndarray, torque_body_nm: np.ndarray, dt: float,
             disturbance: Disturbance) -> None:
        """機体座標の力・トルク（モーターから）と外乱で、dt 秒ぶん進める。"""
        p = self.p
        r = self.rotation()

        # ---- 並進（世界座標）: m a = R F_body + F_外乱 − c v + m g ----
        f_world = r @ force_body_n + disturbance.force_world_n - p.drag_n_per_mps * self.vel_mps
        accel = f_world / p.mass_kg + np.array([0.0, 0.0, -G0])

        # ---- 床 ----
        if self.on_ground:
            if accel[2] > 0.0:
                self.on_ground = False          # 上向きの力が重さを超えた → 離陸
            else:
                accel = np.zeros(3)             # 床が支える（垂直抗力）。横にもすべらない
        if self.on_ground:
            self.vel_mps = np.zeros(3)
            self.omega_rps = np.zeros(3)
            self.accel_mps2 = accel
            return

        # ---- 回転（機体座標）: I ω̇ = τ − ω × (I ω)（オイラーの運動方程式） ----
        inertia = p.inertia_kgm2
        tau = torque_body_nm + disturbance.torque_body_nm - p.angular_drag_nm_per_rps * self.omega_rps
        omega = self.omega_rps
        omega_dot = (tau - np.cross(omega, inertia * omega)) / inertia
        self.omega_rps = omega + omega_dot * dt
        # 姿勢: 機体座標の角速度で dt だけ回す。q_new = q ⊗ exp(ω dt / 2)
        self.quat = quat_mul(self.quat, quat_from_rotvec(self.omega_rps * dt))
        self.quat /= np.linalg.norm(self.quat)

        # ---- 並進を進める ----
        self.vel_mps = self.vel_mps + accel * dt
        self.pos_m = self.pos_m + self.vel_mps * dt
        self.accel_mps2 = accel

        if self.pos_m[2] <= 0.0 and self.vel_mps[2] <= 0.0:
            # 着地: 高さ0で止める（はねない）。姿勢はそのまま、動きは止める
            self.pos_m[2] = 0.0
            self.vel_mps = np.zeros(3)
            self.omega_rps = np.zeros(3)
            self.accel_mps2 = np.zeros(3)
            self.on_ground = True

    def euler_zyx(self) -> np.ndarray:
        """ロール・ピッチ・ヨー [rad]（Z-Y-X、ファームと同じ定義）。グラフの「本当」の線に使う。"""
        w, x, y, z = self.quat
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return np.array([roll, pitch, yaw])


# ---------------------------------------------------------------------------
# 5. IMU
# ---------------------------------------------------------------------------

class Imu:
    """本当の状態から IMU の出力を作る。IMU は重心にあるとみなす（ずれていると回転で余分な加速度が出る）。"""

    def __init__(self, p: ImuParams):
        self.p = p
        self.rng = np.random.default_rng(0)

    def reset(self, rng: np.random.Generator) -> None:
        self.rng = rng

    def read(self, body: RigidBody) -> tuple[np.ndarray, np.ndarray]:
        """(角速度 [rad/s], 比力 [g])、機体座標。

        比力 = 加速度 − 重力加速度（を機体座標に直して g 単位に）。重力加速度は世界の (0, 0, −G0)
        なので、静止なら上向きに +1g、自由落下なら 0。
        """
        p = self.p
        r = body.rotation()
        spec_world = body.accel_mps2 - np.array([0.0, 0.0, -G0])
        accel_g = r.T @ spec_world / G0 + p.accel_bias_g
        gyro = body.omega_rps + p.gyro_bias_rps
        if p.gyro_noise_rps > 0.0:
            gyro = gyro + self.rng.normal(0.0, p.gyro_noise_rps, 3)
        if p.accel_noise_g > 0.0:
            accel_g = accel_g + self.rng.normal(0.0, p.accel_noise_g, 3)
        return gyro, accel_g


# ---------------------------------------------------------------------------
# 組み立て（runner.py が呼ぶ形）
# ---------------------------------------------------------------------------

class Drone:
    """DroneModel の形。部品を決まった順に呼ぶ。"""

    def __init__(self, params: DroneParams | None = None,
                 initial_euler_rad: tuple[float, float, float] | None = None):
        self.params = params or DroneParams()
        self.initial_euler_rad = None if initial_euler_rad is None else np.asarray(initial_euler_rad, float)
        self.motors = Motors(self.params.motor)
        self.battery = Battery(self.params.battery)
        self.body = RigidBody(self.params.body)
        self.imu_sensor = Imu(self.params.imu)

    def reset(self, rng: np.random.Generator) -> None:
        self.motors.reset()
        self.battery.reset()
        self.body.reset(self.initial_euler_rad)
        self.imu_sensor.reset(rng)

    def forces(self) -> tuple[np.ndarray, np.ndarray]:
        """モーターの推力・反トルクを、機体座標の合計の力 [N] とトルク [N·m] にまとめる。

        推力は各モーターの位置で機体の +z 向き。トルク = Σ r_i × F_i + Σ 反トルク_i ẑ。
        r × (0, 0, F) = (y F, −x F, 0) なので、ロールは左（y>0）、ピッチは後ろ（x<0）のモーターで正になる
        （ミキサーの符号表と同じ）。
        """
        f = self.motors.thrust_n()
        pos = self.params.body.motor_pos_m
        force = np.array([0.0, 0.0, f.sum()])
        torque = np.array([np.dot(pos[:, 1], f), -np.dot(pos[:, 0], f),
                           self.motors.reaction_torque_nm().sum()])
        return force, torque

    def step(self, duty: np.ndarray, dt: float, disturbance: Disturbance) -> None:
        self.motors.step(duty, self.battery.v_terminal, dt)       # 1. 前の刻みの端子電圧で回す
        self.battery.update(float(self.motors.current_a().sum()))  # 2. 新しい電流で端子電圧
        force, torque = self.forces()                              # 3. 合計の力とトルク
        self.body.step(force, torque, dt, disturbance)             # 4. 剛体を進める

    def imu(self) -> tuple[np.ndarray, np.ndarray]:
        return self.imu_sensor.read(self.body)

    def battery_voltage(self) -> float:
        return float(self.battery.v_terminal)

    def log(self) -> dict[str, float | np.ndarray]:
        return {
            "true_angle_rad": self.body.euler_zyx(),
            "true_rate_rps": self.body.omega_rps.copy(),
            "pos_m": self.body.pos_m.copy(),
            "vel_mps": self.body.vel_mps.copy(),
            "thrust_g": self.motors.thrust_g.copy(),
            "current_a": float(self.battery.current_a),
            "on_ground": float(self.body.on_ground),
        }
