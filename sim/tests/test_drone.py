"""機体モデル（fcsim/drone.py）の物理の試験。ファームは使わない（最後の符号表の試験だけ読む）。"""
import numpy as np
import pytest

from fcsim import FlightComputer, NO_DISTURBANCE, Disturbance
from fcsim.drone import (G0, GRAM_TO_N, Battery, BatteryParams, BodyParams, Drone, DroneParams, Imu,
                         ImuParams, MotorParams, Motors, RigidBody, quat_from_euler_zyx)
from fcsim.model import euler_zyx_to_matrix

DT = 1.0 / 7519.0


def steps(seconds):
    return int(round(seconds / DT))


def airborne_body(p=None, z=10.0):
    b = RigidBody(p or BodyParams())
    b.pos_m[2] = z
    b.on_ground = False
    return b


# ---- 1. モーター -------------------------------------------------------------

def test_motor_step_response_is_first_order_lag():
    # デューティ0.36・3.7V → 定常 9.4×0.36×3.7 = 12.52g。τ=70ms で 63%、300ms でほぼ定常
    m = Motors(MotorParams())
    duty = np.full(4, 0.36)
    t_ss = 9.4 * 0.36 * 3.7
    for _ in range(steps(0.070)):
        m.step(duty, 3.7, DT)
    assert m.thrust_g == pytest.approx(np.full(4, t_ss * (1 - np.exp(-1))), rel=1e-3)
    for _ in range(steps(0.230)):
        m.step(duty, 3.7, DT)
    assert m.thrust_g == pytest.approx(np.full(4, t_ss * (1 - np.exp(-0.3 / 0.07))), rel=1e-3)


def test_motor_units_current_and_reaction_torque():
    m = Motors(MotorParams())
    m.thrust_g = np.array([12.5, 12.5, 12.5, 12.5])
    assert m.thrust_n() == pytest.approx(np.full(4, 12.5 * G0 * 1e-3))
    # 電流は A/g × g（HW設計書6章: ホバリング 12.5g で約1.1A）
    assert m.current_a() == pytest.approx(np.full(4, 1.11), abs=0.02)
    # 同じ推力なら反トルクは打ち消し合う。時計回り（M1・M4）は機体を +z（反時計回り）に回す
    assert m.reaction_torque_nm().sum() == pytest.approx(0.0, abs=1e-15)
    assert np.sign(m.reaction_torque_nm()) == pytest.approx([+1, -1, -1, +1])


def test_motor_clips_duty():
    m = Motors(MotorParams())
    for _ in range(steps(1.0)):
        m.step(np.array([1.5, -0.2, 0.5, 1.0]), 3.7, DT)
    assert m.thrust_g == pytest.approx([9.4 * 3.7, 0.0, 9.4 * 0.5 * 3.7, 9.4 * 3.7], rel=1e-4)


# ---- 2. 電池 -----------------------------------------------------------------

def test_battery_internal_resistance_drop():
    b = Battery(BatteryParams(v_open_v=3.8, r_internal_ohm=0.1))
    b.update(4.5)
    assert b.v_terminal == pytest.approx(3.8 - 0.45)


def test_drone_uses_previous_terminal_voltage_and_sags():
    p = DroneParams()
    p.battery.r_internal_ohm = 0.1
    d = Drone(p)
    d.reset(np.random.default_rng(0))
    for _ in range(steps(1.0)):
        d.step(np.full(4, 0.4), DT, NO_DISTURBANCE)
    # 定常: V = V0 − R·4·k_i·k·duty·V → V = V0 / (1 + R·4·k_i·k·duty)
    v_expect = 3.8 / (1 + 0.1 * 4 * 0.089 * 9.4 * 0.4)
    assert d.battery_voltage() == pytest.approx(v_expect, rel=1e-3)


# ---- 3. 回転 -----------------------------------------------------------------

def test_quaternion_matches_firmware_euler_definition():
    b = RigidBody(BodyParams())
    for e in [(0.3, -0.2, 1.0), (-0.5, 0.4, -2.5), (0.0, 0.0, 0.0)]:
        b.quat = quat_from_euler_zyx(*e)
        assert b.rotation() == pytest.approx(euler_zyx_to_matrix(*e), abs=1e-12)
        assert b.euler_zyx() == pytest.approx(e, abs=1e-12)


def test_constant_torque_spins_up_with_inertia():
    p = BodyParams()
    b = airborne_body(p)
    tau = np.array([1e-4, 0.0, 0.0])
    up = np.array([0.0, 0.0, p.mass_kg * G0])   # 重さを打ち消して高さを保つ
    for _ in range(steps(0.1)):
        b.step(up, tau, DT, NO_DISTURBANCE)
    assert b.omega_rps[0] == pytest.approx(1e-4 * 0.1 / p.inertia_kgm2[0], rel=1e-3)
    roll = b.euler_zyx()[0]
    assert roll == pytest.approx(0.5 * 1e-4 / p.inertia_kgm2[0] * 0.1 ** 2, rel=1e-2)


def test_torque_free_rotation_conserves_energy_and_momentum():
    # ジャイロ効果（ω × Iω）の符号と積分の精度: 外からトルクが無ければ、回転のエネルギーと
    # 世界座標の角運動量は変わらない。3軸の慣性が違う物体を斜めに回す
    p = BodyParams(inertia_kgm2=np.array([4e-5, 6e-5, 1e-4]))
    b = airborne_body(p)
    b.omega_rps = np.array([3.0, 0.5, 2.0])
    up = np.array([0.0, 0.0, p.mass_kg * G0])

    def energy_and_l(body):
        i = p.inertia_kgm2
        w = body.omega_rps
        return 0.5 * np.dot(i * w, w), body.rotation() @ (i * w)

    e0, l0 = energy_and_l(b)
    for _ in range(steps(2.0)):
        b.step(up, np.zeros(3), DT, NO_DISTURBANCE)
    e1, l1 = energy_and_l(b)
    assert e1 == pytest.approx(e0, rel=2e-3)
    assert l1 == pytest.approx(l0, abs=2e-3 * np.linalg.norm(l0))
    assert np.linalg.norm(b.quat) == pytest.approx(1.0, abs=1e-12)


def test_motor_torques_match_firmware_mixer_sign_table():
    # 機体モデルの「モーター i の推力を上げると、どの向きのトルクが出るか」が、ファームの符号表と同じ
    d = Drone()
    fc = FlightComputer()
    fc.reset()
    sign = fc.mixer_sign()
    for i in range(4):
        d.motors.thrust_g = np.full(4, 10.0)
        _, t0 = d.forces()
        d.motors.thrust_g[i] += 1.0
        _, t1 = d.forces()
        assert np.sign(t1 - t0) == pytest.approx(sign[i]), f"M{i + 1}"


# ---- 4. 並進と床 -------------------------------------------------------------

def test_free_fall_and_specific_force_zero():
    p = BodyParams(drag_n_per_mps=0.0)
    b = airborne_body(p)
    imu = Imu(ImuParams(gyro_noise_rps=0.0, accel_noise_g=0.0, gyro_bias_rps=np.zeros(3)))
    for _ in range(steps(0.5)):
        b.step(np.zeros(3), np.zeros(3), DT, NO_DISTURBANCE)
    assert b.pos_m[2] == pytest.approx(10.0 - 0.5 * G0 * 0.5 ** 2, abs=2e-3)
    _, accel = imu.read(b)
    assert accel == pytest.approx(np.zeros(3), abs=1e-12)


def test_hover_thrust_balances_weight():
    d = Drone()
    d.reset(np.random.default_rng(0))
    d.params.imu.gyro_noise_rps = d.params.imu.accel_noise_g = 0.0
    d.body.on_ground = False
    d.body.pos_m[2] = 1.0
    d.motors.thrust_g = np.full(4, d.params.body.mass_kg * G0 / 4 / GRAM_TO_N)
    force, torque = d.forces()
    d.body.step(force, torque, DT, NO_DISTURBANCE)
    assert d.body.accel_mps2 == pytest.approx(np.zeros(3), abs=1e-9)
    _, accel = d.imu()
    assert accel == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)


def test_floor_holds_until_thrust_exceeds_weight():
    p = BodyParams()
    b = RigidBody(p)
    w = p.mass_kg * G0
    for _ in range(steps(0.5)):
        b.step(np.array([0.0, 0.0, 0.9 * w]), np.array([1e-3, 0.0, 0.0]), DT, NO_DISTURBANCE)
    assert b.on_ground and b.pos_m[2] == 0.0
    assert b.omega_rps == pytest.approx(np.zeros(3))
    for _ in range(steps(0.5)):
        b.step(np.array([0.0, 0.0, 1.1 * w]), np.zeros(3), DT, NO_DISTURBANCE)
    assert not b.on_ground
    assert b.pos_m[2] == pytest.approx(0.5 * 0.1 * G0 * 0.5 ** 2, rel=0.05)


def test_tilted_on_floor_reads_gravity_in_body_axes():
    d = Drone(initial_euler_rad=(0.1, -0.05, 0.3))
    d.reset(np.random.default_rng(0))
    d.params.imu.accel_noise_g = 0.0
    d.step(np.zeros(4), DT, NO_DISTURBANCE)
    _, accel = d.imu()
    assert accel == pytest.approx(euler_zyx_to_matrix(0.1, -0.05, 0.3).T @ [0, 0, 1], abs=1e-12)


def test_world_force_disturbance_pushes_in_world_axes():
    p = BodyParams(drag_n_per_mps=0.0)
    b = airborne_body(p)
    b.quat = quat_from_euler_zyx(0.0, 0.0, np.pi / 2)    # 機首を左（世界の+y）へ
    up = np.array([0.0, 0.0, p.mass_kg * G0])
    for _ in range(steps(0.1)):
        b.step(up, np.zeros(3), DT, Disturbance(force_world_n=np.array([0.01, 0.0, 0.0])))
    assert b.vel_mps == pytest.approx([0.01 / p.mass_kg * 0.1, 0.0, 0.0], rel=1e-3, abs=1e-9)


# ---- 5. IMU ------------------------------------------------------------------

def test_imu_noise_and_bias():
    p = ImuParams()
    imu = Imu(p)
    imu.reset(np.random.default_rng(1))
    b = RigidBody(BodyParams())
    g = np.array([imu.read(b)[0] for _ in range(20000)])
    a = np.array([imu.read(b)[1] for _ in range(20000)])
    assert g.mean(axis=0) == pytest.approx(p.gyro_bias_rps, abs=4 * p.gyro_noise_rps / np.sqrt(20000))
    assert g.std(axis=0) == pytest.approx(np.full(3, p.gyro_noise_rps), rel=0.03)
    assert a.std(axis=0) == pytest.approx(np.full(3, p.accel_noise_g), rel=0.03)
    assert a.mean(axis=0) == pytest.approx([0, 0, 1], abs=1e-3)
