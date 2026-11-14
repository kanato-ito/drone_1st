"""機体モデル（drone.py）で飛ばす、最初の4つの試験（設計メモ 6章「シミュレータ」）。

  1. hover        アーム → 離陸 → 高さ0.5mでホバリング
  2. disturbance  ホバリング中に、ロール・ピッチ・ヨーの順にトルクの短いパルス（外乱）を加え、
                  戻す向きに指令が出て元の姿勢に戻るかを見る。最後に一定のロールのトルク（重心のずれの代わり）
  3. windup       傾いた床の上で 8g/モーター（離陸しない推力）のまま待ってから離陸する。積分を始める
                  しきい値 i_enable_thrust_g が 3g（元の仮の値）と 9g（2026-11-13 決定）とで比べる。
                  3g のときは姿勢推定の時定数 tau_s 0.5秒と2秒（2026-11-13 決定）も比べる
  4. resistance   電池の内部抵抗を 0〜0.15Ω で振り、ホバリングと上昇で電圧が揺れないかを見る

ファームには高さの制御が無い（目標は姿勢と1モーターあたりの推力）。そこで「操縦者の代わり」の
AltitudePilot が、機体モデルの本当の高さを見て目標推力を決める（目のよい人が高さを保つのに相当）。

    cd sim && python -m fcsim.scenarios.flight            # 4つとも。グラフは sim/out/flight_*.png
    cd sim && python -m fcsim.scenarios.flight hover      # 1つだけ
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from fcsim import Controls, Disturbance, NO_DISTURBANCE, Scenario, Simulation
from fcsim.drone import G0, GRAM_TO_N, Drone, DroneParams
from fcsim.plot import plot_log

DEG = np.pi / 180.0
OUT_DIR = Path(__file__).resolve().parents[2] / "out"

T_TAKEOFF = 2.5      # アーム（静止1秒＋バイアス）が済んでから上がり始める時刻 [s]
HOVER_Z_M = 0.5


def ramp01(t: float, t0: float, t1: float) -> float:
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    x = (t - t0) / (t1 - t0)
    return x * x * (3.0 - 2.0 * x)


@dataclass
class AltitudePilot:
    """操縦者の代わり。高さの目標 z_target(t) に向けて、1モーターあたりの目標推力 [g] を決める。

    推力 = ホバリングの推力 + kp × 高さの誤差 − kd × 上下の速さ（PD）。傾いたぶん（1/cos）も足す。
    z_target(t) が None の間は wait_thrust_g（床で待つ推力）を出す。
    """

    drone: Drone
    z_target: Callable[[float], float | None]
    wait_thrust_g: float = 0.0
    kp_g_per_m: float = 8.0
    kd_g_per_mps: float = 6.0
    max_thrust_g: float = 25.0
    attitude: Callable[[float], tuple[float, float, float]] = lambda t: (0.0, 0.0, 0.0)

    @property
    def hover_g(self) -> float:
        return self.drone.params.body.mass_kg * G0 / 4.0 / GRAM_TO_N

    def __call__(self, t: float) -> Controls:
        roll, pitch, yaw = self.attitude(t)
        zt = self.z_target(t)
        if zt is None:
            return Controls(arm=True, roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw, thrust_g=self.wait_thrust_g)
        body = self.drone.body
        tilt = body.rotation()[2, 2]          # 機体z軸の世界z成分 = cos(傾き)
        th = (self.hover_g + self.kp_g_per_m * (zt - body.pos_m[2])
              - self.kd_g_per_mps * body.vel_mps[2]) / max(tilt, 0.5)
        return Controls(arm=True, roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw,
                        thrust_g=float(np.clip(th, 0.0, self.max_thrust_g)))


def takeoff_profile(z: float = HOVER_Z_M, t0: float = T_TAKEOFF, rise_s: float = 1.0):
    return lambda t: None if t < t0 else z * ramp01(t, t0, t0 + rise_s)


@dataclass
class Result:
    name: str
    log: dict
    drone: Drone
    png: Path | None = None


def _run(name: str, drone: Drone, scenario: Scenario, plot: bool, title: str, seed: int = 0) -> Result:
    log = Simulation(drone, seed=seed).run(scenario)
    png = plot_log(log, OUT_DIR / f"flight_{name}.png", title=title) if plot else None
    return Result(name, log, drone, png)


def at(log, t: float) -> int:
    return min(int(np.searchsorted(log["t"], t)), len(log["t"]) - 1)


def window(log, t0: float, t1: float) -> slice:
    return slice(at(log, t0), at(log, t1))


# ---------------------------------------------------------------------------
# 1. 離陸 → ホバリング
# ---------------------------------------------------------------------------

def hover(plot: bool = True, params: DroneParams | None = None, duration_s: float = 8.0) -> Result:
    d = Drone(params)
    pilot = AltitudePilot(d, takeoff_profile())
    return _run("hover", d, Scenario(duration_s, pilot), plot, "1. 離陸してホバリング（0.5m）")


# ---------------------------------------------------------------------------
# 2. 外乱（トルクのパルス、一定のトルク）
# ---------------------------------------------------------------------------

PULSE_NM = 1.0e-3     # 30ms で ロール・ピッチは約0.55rad/s、ヨーは約0.3rad/s の角速度になる大きさ
PULSE_S = 0.030
PULSES = ((5.0, 0), (7.0, 1), (9.0, 2))      # (時刻, 軸)
STEADY_T0, STEADY_T1, STEADY_NM = 11.0, 15.0, 2.0e-4   # ロールに一定のトルク（約0.12g/モーター分）


def disturbance_profile(t: float) -> Disturbance:
    tq = np.zeros(3)
    for t0, axis in PULSES:
        if t0 <= t < t0 + PULSE_S:
            tq[axis] = PULSE_NM
    if STEADY_T0 <= t < STEADY_T1:
        tq[0] += STEADY_NM
    if not tq.any():
        return NO_DISTURBANCE
    return Disturbance(torque_body_nm=tq)


def disturbance(plot: bool = True, params: DroneParams | None = None) -> Result:
    d = Drone(params)
    pilot = AltitudePilot(d, takeoff_profile())
    sc = Scenario(16.0, pilot, disturbance=disturbance_profile)
    return _run("disturbance", d, sc, plot, "2. 外乱: ロール・ピッチ・ヨーのパルス（各+1mN·m・30ms）と一定のロールトルク")


# ---------------------------------------------------------------------------
# 3. 床の上での積分ワインドアップ
# ---------------------------------------------------------------------------

FLOOR_ROLL_RAD = 5.0 * DEG      # 床の傾き（アームできる上限 10° の内側）
WINDUP_TAKEOFF = 6.0


I_ENABLE_OLD_G = 3.0       # 元の仮の値（浮く推力 約12.3g よりずっと下）
I_ENABLE_DECIDED_G = 9.0   # 2026-11-13 決定（浮く推力の7割強）
TAU_OLD_S = 0.5
TAU_DECIDED_S = 2.0        # 2026-11-13 決定（初期値。残りは実機で確かめる）


def windup(i_enable_g: float, tau_s: float, wait_thrust_g: float = 8.0, plot: bool = True,
           params: DroneParams | None = None) -> Result:
    d = Drone(params, initial_euler_rad=(FLOOR_ROLL_RAD, 0.0, 0.0))
    pilot = AltitudePilot(d, takeoff_profile(t0=WINDUP_TAKEOFF), wait_thrust_g=wait_thrust_g)

    def setup(fc):
        fc.set_param("pid.i_enable_thrust_g", i_enable_g)
        fc.set_param("attitude.tau_s", tau_s)

    name = f"windup_ien{i_enable_g:g}g_tau{tau_s:g}s"
    title = (f"3. 傾いた床（ロール+5°）で {wait_thrust_g:g}g/モーター待って離陸"
             f"（i_enable {i_enable_g:g}g、tau_s {tau_s:g}s）")
    return _run(name, d, Scenario(10.0, pilot, fc_setup=setup), plot, title)


# ---------------------------------------------------------------------------
# 4. 電池の内部抵抗
# ---------------------------------------------------------------------------

R_SWEEP_OHM = (0.0, 0.05, 0.10, 0.15)
CLIMB_T = 6.0


V_LOW_FOR_SWEEP = 2.9   # 電圧の揺れだけを見るときの低電圧しきい値（ファームの既定 3.6V だと降下に入るため）


def resistance(r_ohm: float, plot: bool = True, v_open_v: float = 3.8, v_low: float | None = V_LOW_FOR_SWEEP,
               duration_s: float = 10.0) -> Result:
    """v_low=None ならファームの既定の低電圧しきい値のまま（設計メモ7章 #7 で未決）。"""
    params = DroneParams()
    params.battery.r_internal_ohm = r_ohm
    params.battery.v_open_v = v_open_v
    d = Drone(params)

    def z_target(t: float):
        z = takeoff_profile()(t)
        if z is None:
            return None
        return z + 0.5 * ramp01(t, CLIMB_T, CLIMB_T + 0.5)    # 6秒で 0.5m → 1.0m に急いで上がる

    pilot = AltitudePilot(d, z_target)

    def setup(fc):
        if v_low is not None:
            fc.set_param("battery.v_min", min(v_low, fc.param("battery.v_min")))   # params_validate: v_low ≥ v_min
            fc.set_param("battery.v_low", v_low)

    vl = "既定" if v_low is None else f"{v_low:g}V"
    title = f"4. 内部抵抗 {r_ohm:.2f}Ω（開放電圧 {v_open_v:.2f}V、v_low {vl}）でホバリング → 6秒で上昇"
    name = f"resistance_{int(round(r_ohm * 1000)):03d}mohm" + ("_vlow_default" if v_low is None else "")
    return _run(name, d, Scenario(duration_s, pilot, fc_setup=setup), plot, title)


# ---------------------------------------------------------------------------
# まとめて走らせて、数字を出す
# ---------------------------------------------------------------------------

def _summary_hover(r: Result) -> None:
    log = r.log
    armed = log["armed"] > 0.5
    lifted = log["model_pos_m"][:, 2] > 0.01
    w = window(log, 5.0, 8.0)
    true = log["model_true_angle_rad"][w]
    est = log["angle_rad"][w]
    print(f"[hover] アーム {log['t'][armed][0]:.2f}s / 浮上 {log['t'][lifted][0]:.2f}s / "
          f"止まった高さ {log['model_pos_m'][-1, 2]:.3f}m / 非常停止 {int(log['tripped'].max())}")
    print(f"        5〜8秒: 本当の傾き 最大 roll {np.abs(true[:, 0]).max() / DEG:.2f}° pitch "
          f"{np.abs(true[:, 1]).max() / DEG:.2f}° / 推定−本当 最大 "
          f"{np.abs(est[:, :2] - true[:, :2]).max() / DEG:.2f}° / デューティ {log['duty'][-1].round(3)}")


def _summary_disturbance(r: Result) -> None:
    log = r.log
    names = ("roll", "pitch", "yaw")
    for t0, axis in PULSES:
        w = window(log, t0, t0 + 1.5)
        ang = log["model_true_angle_rad"][:, axis] - log["model_true_angle_rad"][at(log, t0 - 0.05), axis]
        first = log["out_torque_g"][at(log, t0 + PULSE_S), axis]
        late = np.abs(ang[window(log, t0 + 1.0, t0 + 2.0)]).max()
        print(f"[disturbance] {names[axis]:5s}: 最大のずれ {ang[w].max() / DEG:+.2f}° / パルス直後の指令 {first:+.2f}g"
              f"（逆向きなら−）/ 1〜2秒後のずれ 最大 {late / DEG:.2f}°")
    w = window(log, STEADY_T1 - 1.0, STEADY_T1)
    print(f"[disturbance] 一定のロールトルク {STEADY_NM * 1e3:.1f}mN·m: 最後の1秒の roll 平均 "
          f"{log['model_true_angle_rad'][w, 0].mean() / DEG:+.2f}°、I項 {log['rate_integ_g'][w, 0].mean():+.3f}g")


def _summary_windup(r: Result) -> None:
    log = r.log
    i_before = log["rate_integ_g"][at(log, WINDUP_TAKEOFF - 0.01), 0]
    w = window(log, WINDUP_TAKEOFF, WINDUP_TAKEOFF + 3.0)
    roll = log["model_true_angle_rad"][w, 0]
    err = np.abs(log["angle_rad"][w, 0] - roll).max()
    print(f"[{r.name}] 離陸の直前の I項 roll {i_before:+.3f}g / 離陸後3秒の roll 最小 {roll.min() / DEG:+.2f}° "
          f"最大 {roll.max() / DEG:+.2f}° / 最後 {log['model_true_angle_rad'][-1, 0] / DEG:+.2f}° / "
          f"推定のずれ 最大 {err / DEG:.1f}° / 横に流れた {np.abs(log['model_pos_m'][-1, :2]).max():.1f}m")


def _summary_resistance(r: Result) -> None:
    log = r.log
    w = window(log, 4.0, CLIMB_T)
    wc = window(log, CLIMB_T, 10.0)
    v = log["battery_v_true"]
    z = log["model_pos_m"][:, 2]
    print(f"[{r.name}] ホバリングの端子電圧 {v[w].mean():.3f}V（揺れ ±{v[w].std():.4f}V）/ "
          f"ファームの電圧 {log['batt_v'][w].mean():.3f}V / 上昇中の最低 {v[wc].min():.3f}V / "
          f"高さ 6秒 {z[at(log, CLIMB_T)]:.3f}m → 10秒 {z[-1]:.3f}m / 傾き最大 "
          f"{np.abs(log['model_true_angle_rad'][:, :2]).max() / DEG:.2f}°")


def run_all(names: list[str] | None = None) -> None:
    names = names or ["hover", "disturbance", "windup", "resistance"]
    if "hover" in names:
        r = hover()
        _summary_hover(r)
        print(f"        グラフ: {r.png}")
    if "disturbance" in names:
        r = disturbance()
        _summary_disturbance(r)
        print(f"        グラフ: {r.png}")
    if "windup" in names:
        for ien, tau in ((I_ENABLE_OLD_G, TAU_OLD_S), (I_ENABLE_OLD_G, TAU_DECIDED_S),
                         (I_ENABLE_DECIDED_G, TAU_DECIDED_S)):
            r = windup(ien, tau)
            _summary_windup(r)
            print(f"        グラフ: {r.png}")
    if "resistance" in names:
        for ohm in R_SWEEP_OHM:
            r = resistance(ohm)
            _summary_resistance(r)
        print(f"        グラフ: {OUT_DIR}/flight_resistance_*.png")


if __name__ == "__main__":
    run_all(sys.argv[1:] or None)
