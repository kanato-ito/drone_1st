"""つなぎ込み（ファーム ⇄ ポートの偽物 ⇄ runner）の試験。物理は使わない（BenchModel）。

先にライブラリをビルドしておくこと:  cmake -S sim -B sim/build && cmake --build sim/build
"""
import numpy as np
import pytest

from fcsim import (PID_RATE, ROLL, BenchModel, Controls, FlightComputer, Scenario, Simulation,
                   SAMPLES_PER_CYCLE)

DEG = np.pi / 180.0


@pytest.fixture(scope="module")
def fc():
    return FlightComputer()


def ramp(t, t0, t1):
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    x = (t - t0) / (t1 - t0)
    return x * x * (3 - 2 * x)


def at(log, t):
    return int(np.searchsorted(log["t"], t))


def test_sample_period_matches_firmware(fc):
    # ファームの imu_config_default（実測ODR 7519Hz）と同じ刻みで物理を進める
    assert fc.sample_period_s == pytest.approx(1 / 7519.0, rel=1e-6)


def test_one_cycle_reads_all_pending_samples(fc):
    fc.reset()
    for _ in range(SAMPLES_PER_CYCLE):
        fc.push_imu([0, 0, 0], [0, 0, 1])
    assert fc.imu_pending() == SAMPLES_PER_CYCLE
    assert fc.step(1000) == SAMPLES_PER_CYCLE
    assert fc.imu_pending() == 0
    assert fc.step(2000) == 0   # 空なら何もしない
    assert fc.wdt_feed_count() == 1


def test_arms_only_after_still_and_level(fc):
    log = Simulation(BenchModel(), fc).run(
        Scenario(duration_s=2.0, controls=lambda t: Controls(arm=True, thrust_g=12.5)))
    armed = log["armed"] > 0.5
    t_arm = log["t"][armed][0]
    # 静止が still_hold_ms(1000ms) 続いてから。窓の立ち上がりを含めて 1〜1.6 秒
    assert 1.0 <= t_arm <= 1.6
    assert np.all(log["duty"][~armed] == 0.0)
    assert np.all(np.abs(log["angle_rad"][armed, :2]) < 0.2 * DEG)


def test_does_not_arm_when_tilted(fc):
    model = BenchModel(lambda t: (20 * DEG, 0.0, 0.0))   # アーム許容10°より傾いている
    log = Simulation(model, fc).run(
        Scenario(duration_s=2.5, controls=lambda t: Controls(arm=True, thrust_g=12.5)))
    assert not (log["armed"] > 0.5).any()


def test_tilt_is_estimated_and_command_pushes_back(fc):
    model = BenchModel(lambda t: (10 * DEG * ramp(t, 2.0, 2.3), 0.0, 0.0))
    log = Simulation(model, fc).run(
        Scenario(duration_s=3.0, controls=lambda t: Controls(arm=True, thrust_g=12.5)))
    i = at(log, 2.9)
    assert log["angle_rad"][i, 0] == pytest.approx(10 * DEG, abs=0.3 * DEG)
    torque = log["out_torque_g"][i, 0]
    assert torque < 0.0   # 目標0°に対して +10° → 戻す向き（負）
    # ミキサーが符号表どおりに配っていること: 各モーターの推力の平均からのずれ ∝ 符号表のロール列
    sign = fc.mixer_sign()
    dev = log["motor_g"][i] - log["motor_g"][i].mean()
    assert np.all(np.sign(dev) == np.sign(sign[:, ROLL] * torque))


def test_disarm_stops_motors_and_unlocks(fc):
    log = Simulation(BenchModel(), fc).run(
        Scenario(duration_s=2.5, controls=lambda t: Controls(arm=t < 2.0, thrust_g=12.5)))
    after = log["t"] > 2.01
    assert np.all(log["duty"][after] == 0.0)
    assert np.all(log["armed"][after] == 0)


def test_battery_voltage_goes_through_adc(fc):
    log = Simulation(BenchModel(battery_v=3.62), fc).run(
        Scenario(duration_s=0.5, controls=lambda t: Controls()))
    # 12bit ADC（分圧比2、基準3.3V）の1カウントは約1.6mV
    assert log["batt_raw_v"][-1] == pytest.approx(3.62, abs=0.002)


def test_imu_mount_rotates_into_board_axes(fc):
    # 機体 x前・y左・z上、基板 x右・y前・z上 のときの 機体→基板 の回転
    mount = np.array([[0.0, -1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0]])
    model = BenchModel(lambda t: (10 * DEG * ramp(t, 0.2, 0.5), 0.0, 0.0))   # 機体のロール +10°
    log = Simulation(model, fc, imu_mount=mount).run(
        Scenario(duration_s=1.0, controls=lambda t: Controls()))
    i = at(log, 0.95)
    # ファームはまだ基板の軸で計算する（axis_map 未実装）ので、機体のロールは基板のy軸まわり＝ファームのピッチに出る
    assert log["angle_rad"][i, 1] == pytest.approx(10 * DEG, abs=0.5 * DEG)
    assert abs(log["angle_rad"][i, 0]) < 0.5 * DEG


def test_left_handed_mount_is_rejected(fc):
    with pytest.raises(ValueError):
        Simulation(BenchModel(), fc, imu_mount=np.diag([1.0, 1.0, -1.0]))


def test_pwm_delay_holds_old_duty(fc):
    class Recorder(BenchModel):
        def step(self, duty, dt, disturbance):
            self.seen.append(np.array(duty))
            super().step(duty, dt, disturbance)

        def reset(self, rng):
            self.seen = []
            super().reset(rng)

    m = Recorder()
    Simulation(m, fc, pwm_delay_samples=4).run(
        Scenario(duration_s=1.6, controls=lambda t: Controls(arm=True, thrust_g=12.5)))
    seen = np.array(m.seen)
    first_on = int(np.argmax(seen[:, 0] > 0))
    # アームした周期（8の倍数のステップ k）から4ステップ遅れて、k+5 番目の step 呼び出しで初めて効く
    assert (first_on - 4) % SAMPLES_PER_CYCLE == 0


def test_params_validation(fc):
    fc.reset()
    with pytest.raises(ValueError):
        fc.set_pid_gains(PID_RATE, ROLL, 1.0, 0.0, 0.0, 10.0, 5.0, 50.0)   # i_limit > out_limit
    with pytest.raises(ValueError):
        fc.set_mixer_sign(np.ones((4, 3)))
    with pytest.raises(ValueError):
        fc.set_param("no.such.param", 1.0)
    fc.set_param("battery.lpf_tau_s", 0.2)
    assert fc.param("battery.lpf_tau_s") == pytest.approx(0.2)
    fc.set_pid_gains(PID_RATE, ROLL, 1.2, 0.5, 0.01, 1.0, 6.0, 40.0)
    assert fc.pid_gains(PID_RATE, ROLL)["kp"] == pytest.approx(1.2)
    fc.reset()
    assert fc.param("battery.lpf_tau_s") == pytest.approx(1.0)   # reset で既定値に戻る
