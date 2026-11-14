"""機体モデル＋ファームで飛ばす試験（最初の4つ、fcsim/scenarios/flight.py）。数十秒かかる。

ここで見るのは「いまのファームとゲイン（params.c の仮の値）で、こう振る舞う」という事実。
ゲインや仕組みを変えて結果が変わったら、それがわかるように数字を書いておく。
"""
import numpy as np
import pytest

from fcsim.drone import DEG
from fcsim.scenarios import flight as F


def tilt_deg(log, sl):
    return np.abs(log["model_true_angle_rad"][sl, :2]).max() / DEG


@pytest.fixture(scope="module")
def hover():
    return F.hover(plot=False).log


@pytest.fixture(scope="module")
def dist():
    return F.disturbance(plot=False).log


def test_arm_takeoff_hover(hover):
    log = hover
    armed = log["armed"] > 0.5
    assert armed.any() and log["t"][armed][0] < F.T_TAKEOFF
    assert log["tripped"].max() == 0
    assert log["model_pos_m"][-1, 2] == pytest.approx(F.HOVER_Z_M, abs=0.02)
    w = F.window(log, 5.0, 8.0)
    assert tilt_deg(log, w) < 1.0
    # 姿勢推定は本当の姿勢に合っている（ロール・ピッチ）
    est, true = log["angle_rad"][w, :2], log["model_true_angle_rad"][w, :2]
    assert np.abs(est - true).max() < 1.0 * DEG


@pytest.mark.parametrize("t0,axis", F.PULSES)
def test_disturbance_pulse_is_pushed_back(dist, t0, axis):
    log = dist
    assert log["tripped"].max() == 0
    # パルスは +向き → 直後の指令はその軸で −（戻す向き）。他の軸の指令は小さい
    cmd = log["out_torque_g"][F.at(log, t0 + F.PULSE_S)]
    assert cmd[axis] < 0
    assert np.abs(np.delete(cmd, axis)).max() < 0.2 * abs(cmd[axis])
    ang = log["model_true_angle_rad"][F.window(log, t0 - 0.05, t0 + 1.0), axis]
    ang = ang - ang[0]
    assert 0 < ang.max() / DEG < (10.0 if axis == 2 else 5.0)


@pytest.mark.parametrize("t0,axis", F.PULSES[:2])
def test_roll_pitch_settle_after_pulse(dist, t0, axis):
    ang = dist["model_true_angle_rad"][F.window(dist, t0 + 1.0, t0 + 2.0), axis]
    assert np.abs(ang).max() < 0.5 * DEG


@pytest.mark.xfail(strict=True, reason=(
    "2026-11-09: いまのヨーのゲイン（内側 kp 0.5）では、反トルクの係数 k_q=0.005m（見積もり）のとき"
    "ヨーが約3.7秒周期でほとんど減らずに揺れ続ける（k_q=0.003 なら広がる）。ヨーは反トルクでしか回せず"
    "効きが弱いので、内側 kp を 2 程度にすると収まる。ゲインを直したら xfail を外す"))
def test_yaw_settles_after_pulse(dist):
    t0 = F.PULSES[2][0]
    ang = dist["model_true_angle_rad"][:, 2] - dist["model_true_angle_rad"][F.at(dist, t0 - 0.05), 2]
    assert np.abs(ang[F.window(dist, t0 + 2.0, t0 + 2.0 + 2.0)]).max() < 1.0 * DEG


def test_steady_torque_is_taken_by_integral(dist):
    log = dist
    w = F.window(log, F.STEADY_T1 - 1.0, F.STEADY_T1)
    # 一定のトルクを打ち消す差 = τ ÷ (4 × アーム × 1gの力)。内側のI項がそれを持つ
    need_g = F.STEADY_NM / (4 * 0.04243 * 9.80665e-3)
    assert log["rate_integ_g"][w, 0].mean() == pytest.approx(-need_g, rel=0.2)
    assert abs(log["model_true_angle_rad"][w, 0].mean()) < 1.0 * DEG


@pytest.fixture(scope="module")
def windup_logs():
    return {
        "old": F.windup(F.I_ENABLE_OLD_G, F.TAU_OLD_S, plot=False).log,
        "old_tau2": F.windup(F.I_ENABLE_OLD_G, F.TAU_DECIDED_S, plot=False).log,
        "decided": F.windup(F.I_ENABLE_DECIDED_G, F.TAU_DECIDED_S, plot=False).log,
    }


def _integ_before_takeoff(log):
    return log["rate_integ_g"][F.at(log, F.WINDUP_TAKEOFF - 0.01), 0]


def _after_takeoff(log):
    return F.window(log, F.WINDUP_TAKEOFF, F.WINDUP_TAKEOFF + 3.0)


def test_windup_with_old_threshold(windup_logs):
    # しきい値 3g のまま床で 8g 待つと、積分は上限（i_limit 1.0g）に張りつき、
    # 離陸の直後に反対へ大きく傾く（2026-11-09: 約15°）。決定を変える理由になった現象
    log = windup_logs["old"]
    assert _integ_before_takeoff(log) == pytest.approx(-1.0, abs=1e-3)
    assert -log["model_true_angle_rad"][_after_takeoff(log), 0].min() / DEG > 5.0
    assert log["tripped"].max() == 0


def test_no_windup_with_decided_threshold(windup_logs):
    # 2026-11-13 決定: しきい値 9g（浮く推力の7割強）なら床の上で積分がたまらず、離陸で傾かない
    log = windup_logs["decided"]
    assert _integ_before_takeoff(log) == pytest.approx(0.0, abs=1e-6)
    assert -log["model_true_angle_rad"][_after_takeoff(log), 0].min() / DEG < 1.0
    assert log["tripped"].max() == 0


def test_longer_tau_is_fooled_less_while_drifting(windup_logs):
    # 傾いたまま横に流れている間は、加速度センサーが「水平」に近い値を出す（比力は推力の向き＝機体のz軸）。
    # 相補フィルタはそれに引かれる。tau_s 0.5s で約11°、2s で約4° ずれる（2026-11-09〜13）
    def err(log):
        w = _after_takeoff(log)
        return np.abs(log["angle_rad"][w, 0] - log["model_true_angle_rad"][w, 0]).max() / DEG

    assert err(windup_logs["old"]) > 5.0
    assert err(windup_logs["old_tau2"]) < 0.5 * err(windup_logs["old"])


@pytest.mark.parametrize("r_ohm", [0.0, 0.15])
def test_internal_resistance_sags_without_oscillation(r_ohm):
    # 電圧の揺れだけを見るため、低電圧のしきい値は下げてある（F.V_LOW_FOR_SWEEP）
    log = F.resistance(r_ohm, plot=False).log
    w = F.window(log, 4.0, F.CLIMB_T)
    v = log["battery_v_true"][w]
    i_hover = log["model_current_a"][w].mean()
    assert v.mean() == pytest.approx(3.8 - r_ohm * i_hover, abs=2e-3)
    assert v.std() < 0.01                      # 電圧と推力の補正が揺れ合っていない
    assert log["model_pos_m"][-1, 2] == pytest.approx(1.0, abs=0.02)
    assert tilt_deg(log, slice(None)) < 1.0


def test_default_v_low_starts_descent_while_hovering():
    # 設計メモ7章 #7（未決）: いまの v_low=3.6V のままだと、内部抵抗 0.05Ω でもホバリングの電圧降下だけで
    # 電池低下と判定され、非常時降下に入る。電池が届いて内部抵抗を測ったら、しきい値か判定のしかたを決める
    from fcsim import FS_EMERGENCY_DESCENT, FS_EMERGENCY_STOP
    from fcsim.fc import DESCENT_BATTERY_LOW

    log = F.resistance(0.05, plot=False, v_low=None).log
    st = log["state"]
    i = int(np.argmax(st == FS_EMERGENCY_DESCENT))
    assert st[i] == FS_EMERGENCY_DESCENT
    assert log["descent_reason"][i] == DESCENT_BATTERY_LOW
    assert log["t"][i] < F.CLIMB_T                      # 上昇の前、ホバリング中に
    assert log["battery_v_true"][i] > 3.5               # 電池そのものはまだ十分ある（開放電圧 3.8V）
    assert st[-1] == FS_EMERGENCY_STOP                  # 降りたあとは止まる
    assert log["model_pos_m"][-1, 2] == 0.0
    # 注: 自作版の床はぶつかったときの衝撃を作らないので、着地の検出（衝撃→静か）は働かず、
    # 降下の上限時間（3秒）で止まる。着地の検出は MuJoCo 版で確かめる（設計メモ6章）
