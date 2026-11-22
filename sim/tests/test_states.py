"""本物の control_loop（ステップ③b）の状態機械と安全の仕組みを、シミュの上で確かめる。

ファームの1周期は control_loop_run_once、操縦パケットは本物のメールボックス、旗は本物の ipc_flags を通る。
前半は fc を直接回す小さな試験、後半は機体モデル（drone.py）で飛ばしながらの試験。
"""
import numpy as np
import pytest

from fcsim import (Controls, FlightComputer, Scenario, Simulation, SAMPLES_PER_CYCLE, FS_WAIT_COMM, FS_STANDBY,
                   FS_FLIGHT, FS_EMERGENCY_DESCENT, FS_EMERGENCY_STOP)
from fcsim.drone import DEG, Drone
from fcsim.fc import (DESCENT_COMM_LOST, ESTOP_IMU_TIMEOUT, ESTOP_OPERATOR, ESTOP_TILT_EXCEEDED, FLAG_ARM_REQ,
                      FLAG_ESTOP)
from fcsim.model import BenchModel
from fcsim.scenarios import flight as F

LEVEL_ACCEL = (0.0, 0.0, 1.0)   # 基板の軸でも z は上


@pytest.fixture(scope="module")
def fc():
    return FlightComputer()


class Bench:
    """fc を直接1周期ずつ回す（水平に静止した IMU のサンプルを積む）。"""

    def __init__(self, fc):
        self.fc = fc
        fc.reset(battery_v=3.8)
        self.t_us = 0

    def cycle(self, n_samples=SAMPLES_PER_CYCLE, int1=True, packet=None):
        for _ in range(n_samples):
            self.fc.push_imu((0.0, 0.0, 0.0), LEVEL_ACCEL)
        self.t_us += 1064
        self.fc.set_time_us(self.t_us)
        if packet is not None:
            self.fc.send_packet(**packet)
        again = self.fc.step(int1)
        return self.fc.telemetry(), again

    def run(self, n, **kw):
        tel = None
        for _ in range(n):
            tel, _ = self.cycle(**kw)
        return tel

    def to_standby_and_still(self):
        # パケットが届くと通信待機 → 飛行指示待機。静止1秒でバイアスが求まる
        return self.run(1500, packet=dict(throttle=0.0, flags=0))

    def arm(self, throttle=0.0):
        self.cycle(packet=dict(throttle=throttle, flags=0))
        return self.run(5, packet=dict(throttle=throttle, flags=FLAG_ARM_REQ))


# ---- 1周期とポート ------------------------------------------------------------

def test_one_cycle_reads_all_pending_samples(fc):
    b = Bench(fc)
    tel, again = b.cycle()
    assert tel.batch_count == SAMPLES_PER_CYCLE and fc.imu_pending() == 0 and not again
    tel, _ = b.cycle(n_samples=0)            # INT1は来たのにサンプル0個
    assert tel.batch_count == 0 and tel.no_sample_count == 1
    assert fc.wdt_feed_count() == 2           # どの周期でも餌をやる
    assert fc.wdt_timeout_ms() == 100         # 初期化の最後にWDTを開始（params の既定）


def test_waits_for_comm_then_standby(fc):
    b = Bench(fc)
    tel = b.run(10)
    assert tel.state == FS_WAIT_COMM          # パケットが来るまでは通信待機
    tel = b.run(1, packet=dict(throttle=0.0, flags=0))
    assert tel.state == FS_STANDBY


def test_arm_needs_rising_edge_zero_throttle_and_bias(fc):
    b = Bench(fc)
    # 待機に着く前から押しっぱなしのボタンではアームしない（0→1 の変化が要る）
    tel = b.run(1500, packet=dict(throttle=0.0, flags=FLAG_ARM_REQ))
    assert tel.state == FS_STANDBY and tel.bias_ready
    # スロットルが 0 でなければアームしない（arm_throttle_tol 0.05）
    tel = b.arm(throttle=0.2)
    assert tel.state == FS_STANDBY
    tel = b.arm(throttle=0.0)
    assert tel.state == FS_FLIGHT


def test_no_arm_before_bias_is_ready(fc):
    b = Bench(fc)
    b.run(200, packet=dict(throttle=0.0, flags=0))   # 静止 0.2 秒: バイアスはまだ
    tel = b.arm()
    assert not tel.bias_ready and tel.state == FS_STANDBY


def test_operator_estop_flag_latches(fc):
    b = Bench(fc)
    b.to_standby_and_still()
    b.arm()
    tel = b.run(3, packet=dict(throttle=0.4, flags=FLAG_ESTOP))
    assert tel.state == FS_EMERGENCY_STOP and tel.estop_reason == ESTOP_OPERATOR
    assert np.all(fc.duty() == 0.0)
    tel = b.run(50, packet=dict(throttle=0.0, flags=0))   # 旗を下ろしても抜けない（電源の入れ直しだけ）
    assert tel.state == FS_EMERGENCY_STOP


# ---- IMU の異常 ---------------------------------------------------------------

def flying(b):
    b.to_standby_and_still()
    b.arm()
    tel = b.run(20, packet=dict(throttle=0.45, flags=0))
    assert tel.state == FS_FLIGHT and np.all(b.fc.duty() > 0)
    return tel


def test_no_sample_cycles_hold_command_then_imu_timeout(fc):
    b = Bench(fc)
    flying(b)
    duty_before = fc.duty().copy()
    tel = b.run(15, n_samples=0)              # 15周期: 前回の指令を保つ
    assert tel.state == FS_FLIGHT and np.allclose(fc.duty(), duty_before)
    tel = b.run(1, n_samples=0)               # 16周期目で IMU途絶 → 非常停止
    assert tel.state == FS_EMERGENCY_STOP and tel.estop_reason == ESTOP_IMU_TIMEOUT
    assert np.all(fc.duty() == 0.0)


def test_missing_int1_is_imu_timeout(fc):
    b = Bench(fc)
    flying(b)
    tel, _ = b.cycle(int1=False)              # INT1が来ない周期（実機では5msのタイムアウト）
    assert tel.state == FS_EMERGENCY_STOP and tel.estop_reason == ESTOP_IMU_TIMEOUT


def test_three_read_errors_are_imu_timeout(fc):
    b = Bench(fc)
    flying(b)
    fc.fail_imu_reads(2)
    tel = b.run(2)
    assert tel.state == FS_FLIGHT             # 2回までは続ける
    fc.fail_imu_reads(3)
    tel = b.run(3)
    assert tel.state == FS_EMERGENCY_STOP and tel.estop_reason == ESTOP_IMU_TIMEOUT


def test_int1_stuck_high_latches(fc):
    b = Bench(fc)
    flying(b)
    fc.set_int1_pin_high(True)
    # 読み終えてもピンが High → run_once が「すぐもう一度」を返す。読み直しは空（ウォーターマーク未満）
    tel, again = b.cycle()
    assert again
    n = 1
    while again and n < 20:
        tel, again = b.cycle(n_samples=0)
        n += 1
    assert tel.int1_stuck and not again       # 8回続いたらラッチし、もう読み直さない
    assert tel.state == FS_EMERGENCY_STOP and tel.estop_reason == ESTOP_IMU_TIMEOUT
    fc.set_int1_pin_high(False)


# ---- 機体モデルで飛ばしながら -------------------------------------------------

def test_comm_lost_in_flight_descends_and_stops():
    d = Drone()
    pilot = F.AltitudePilot(d, F.takeoff_profile())
    sc = Scenario(9.0, pilot, link_up=lambda t: t < 5.0)
    log = Simulation(d).run(sc)
    st = log["state"]
    i = int(np.argmax(st == FS_EMERGENCY_DESCENT))
    assert st[i] == FS_EMERGENCY_DESCENT and log["descent_reason"][i] == DESCENT_COMM_LOST
    # comm_timeout_ms（500ms）たってから降下に入る
    assert 5.4 < log["t"][i] < 5.7
    assert np.abs(log["model_true_angle_rad"][i:, :2]).max() < 2 * DEG   # 降下中も姿勢は保つ（PID）
    assert log["model_pos_m"][-1, 2] == 0.0 and st[-1] == FS_EMERGENCY_STOP


def test_tilt_beyond_estop_limit_stops_motors():
    # 試験台で 70° まで傾ける（非常停止は 60°）
    model = BenchModel(lambda t: (70 * DEG * F.ramp01(t, 2.5, 3.5), 0.0, 0.0))
    log = Simulation(model).run(Scenario(4.0, lambda t: Controls(arm=True, thrust_g=12.5)))
    st = log["state"]
    assert (st == FS_FLIGHT).any()
    i = int(np.argmax(st == FS_EMERGENCY_STOP))
    assert log["estop_reason"][i] == ESTOP_TILT_EXCEEDED
    assert log["angle_rad"][i, 0] == pytest.approx(60 * DEG, abs=3 * DEG)
    assert np.all(log["duty"][i:] == 0.0)


def test_disarm_from_flight_returns_to_standby():
    d = Drone()
    pilot = F.AltitudePilot(d, F.takeoff_profile())

    def controls(t):
        c = pilot(t)
        if t > 5.0:
            c.arm = False
        return c

    log = Simulation(d).run(Scenario(5.5, controls))
    assert log["state"][-1] == FS_STANDBY and np.all(log["duty"][-1] == 0.0)
