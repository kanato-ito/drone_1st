"""ファーム（libfcsim）を ctypes で呼ぶ薄い包み。

C側の関数の説明は sim/c/fcsim.h にある。ここは型をそろえて、numpy で受け渡しやすくするだけ。
ファームの状態は静的変数1組なので、1プロセスで同時に動かせる FlightComputer は1つだけ。
"""
from __future__ import annotations

import ctypes as C
import os
import sys
from pathlib import Path

import numpy as np

_SIM_DIR = Path(__file__).resolve().parent.parent


def _default_lib_path() -> Path:
    if sys.platform == "darwin":
        name = "libfcsim.dylib"
    elif sys.platform.startswith("win"):
        name = "fcsim.dll"
    else:
        name = "libfcsim.so"
    return _SIM_DIR / "build" / name


class Telemetry(C.Structure):
    """fcsim.h の fcsim_telemetry_t と同じ並び。片方を変えたらもう片方も直す（大きさは起動時に確かめる）。"""

    _fields_ = [
        ("armed", C.c_int32),
        ("tripped", C.c_int32),
        ("still", C.c_int32),
        ("bias_ready", C.c_int32),
        ("batch_count", C.c_int32),
        ("cycle", C.c_uint32),
        ("angle_rad", C.c_float * 3),
        ("rate_rps", C.c_float * 3),
        ("accel_g", C.c_float * 3),
        ("bias_rps", C.c_float * 3),
        ("target_rad", C.c_float * 3),
        ("target_thrust_g", C.c_float),
        ("rate_target_rps", C.c_float * 3),
        ("out_thrust_g", C.c_float),
        ("out_torque_g", C.c_float * 3),
        ("rate_integ_g", C.c_float * 3),
        ("motor_g", C.c_float * 4),
        ("mix_saturated", C.c_int32),
        ("mix_yaw_scale", C.c_float),
        ("mix_shift_g", C.c_float),
        ("mix_rp_scale", C.c_float),
        ("batt_v", C.c_float),
        ("batt_raw_v", C.c_float),
        ("duty", C.c_float * 4),
    ]

    def as_dict(self) -> dict[str, float | np.ndarray]:
        out: dict[str, float | np.ndarray] = {}
        for name, ctype in self._fields_:
            v = getattr(self, name)
            if hasattr(ctype, "_length_"):
                out[name] = np.array(v[:], dtype=np.float64)
            else:
                out[name] = float(v)
        return out


_F3 = C.c_float * 3
_F4 = C.c_float * 4
_F6 = C.c_float * 6
_I8x12 = C.c_int8 * 12

PID_ANGLE, PID_RATE = 0, 1
ROLL, PITCH, YAW = 0, 1, 2


class FlightComputer:
    """PC上のファーム1台ぶん。"""

    def __init__(self, lib_path: str | os.PathLike | None = None):
        path = Path(lib_path) if lib_path else Path(os.environ.get("FCSIM_LIB", _default_lib_path()))
        if not path.exists():
            raise FileNotFoundError(
                f"{path} がありません。先にビルドしてください:\n"
                "  cmake -S sim -B sim/build -DCMAKE_BUILD_TYPE=Release && cmake --build sim/build"
            )
        self._lib = C.CDLL(str(path))
        self._declare()
        if self._lib.fcsim_telemetry_size() != C.sizeof(Telemetry):
            raise RuntimeError("fcsim_telemetry_t と Telemetry の並びが食い違っています（fcsim.h と fc.py）")
        self._tel = Telemetry()
        self.reset()

    def _declare(self) -> None:
        L = self._lib
        L.fcsim_imu_push.argtypes = [_F3, _F3]
        L.fcsim_imu_push.restype = None
        L.fcsim_imu_pending.restype = C.c_int32
        L.fcsim_imu_sample_period_s.restype = C.c_float
        L.fcsim_set_time_us.argtypes = [C.c_uint64]
        L.fcsim_set_batt_raw.argtypes = [C.c_uint16]
        L.fcsim_set_batt_voltage.argtypes = [C.c_float]
        L.fcsim_get_duty.argtypes = [_F4]
        L.fcsim_wdt_feed_count.restype = C.c_uint32
        L.fcsim_init.restype = C.c_int32
        L.fcsim_set_target.argtypes = [C.c_float] * 4
        L.fcsim_set_arm_request.argtypes = [C.c_int32]
        L.fcsim_step.argtypes = [C.c_uint32]
        L.fcsim_step.restype = C.c_int32
        L.fcsim_get_telemetry.argtypes = [C.POINTER(Telemetry)]
        L.fcsim_telemetry_size.restype = C.c_int32
        L.fcsim_set_pid_gains.argtypes = [C.c_int32, C.c_int32, _F6]
        L.fcsim_set_pid_gains.restype = C.c_int32
        L.fcsim_get_pid_gains.argtypes = [C.c_int32, C.c_int32, _F6]
        L.fcsim_get_pid_gains.restype = C.c_int32
        L.fcsim_set_mixer_sign.argtypes = [_I8x12]
        L.fcsim_set_mixer_sign.restype = C.c_int32
        L.fcsim_get_mixer_sign.argtypes = [_I8x12]
        L.fcsim_set_param.argtypes = [C.c_char_p, C.c_float]
        L.fcsim_set_param.restype = C.c_int32
        L.fcsim_get_param.argtypes = [C.c_char_p, C.POINTER(C.c_float)]
        L.fcsim_get_param.restype = C.c_int32

    # ---- 起動・1周期 ----

    def reset(self) -> None:
        """起動処理からやり直す（params も既定値に戻る）。"""
        rc = self._lib.fcsim_init()
        if rc != 0:
            raise RuntimeError(f"fcsim_init が失敗しました ({rc})")

    def step(self, int1_timestamp_us: int) -> int:
        """1周期回す。戻り値は読んだサンプル数。"""
        return int(self._lib.fcsim_step(int1_timestamp_us & 0xFFFFFFFF))

    def telemetry(self) -> Telemetry:
        self._lib.fcsim_get_telemetry(C.byref(self._tel))
        return self._tel

    # ---- ポートの偽物 ----

    @property
    def sample_period_s(self) -> float:
        return float(self._lib.fcsim_imu_sample_period_s())

    def push_imu(self, gyro_rps, accel_g) -> None:
        """IMU基板の軸で1サンプル積む。"""
        self._lib.fcsim_imu_push(_F3(*map(float, gyro_rps)), _F3(*map(float, accel_g)))

    def imu_pending(self) -> int:
        return int(self._lib.fcsim_imu_pending())

    def set_time_us(self, now_us: int) -> None:
        self._lib.fcsim_set_time_us(int(now_us))

    def set_battery_voltage(self, volts: float) -> None:
        self._lib.fcsim_set_batt_voltage(float(volts))

    def set_battery_raw(self, raw: int) -> None:
        self._lib.fcsim_set_batt_raw(int(raw))

    def duty(self) -> np.ndarray:
        buf = _F4()
        self._lib.fcsim_get_duty(buf)
        return np.array(buf[:], dtype=np.float64)

    def wdt_feed_count(self) -> int:
        return int(self._lib.fcsim_wdt_feed_count())

    # ---- 操縦（メールボックスの代わり） ----

    def set_target(self, roll_rad: float = 0.0, pitch_rad: float = 0.0, yaw_rad: float = 0.0,
                   thrust_g: float = 0.0) -> None:
        self._lib.fcsim_set_target(float(roll_rad), float(pitch_rad), float(yaw_rad), float(thrust_g))

    def set_arm_request(self, arm: bool) -> None:
        self._lib.fcsim_set_arm_request(1 if arm else 0)

    # ---- パラメータ ----

    def set_pid_gains(self, stage: int, axis: int, kp: float, ki: float, kd: float,
                      i_limit: float, out_limit: float, d_lpf_hz: float) -> None:
        g = _F6(kp, ki, kd, i_limit, out_limit, d_lpf_hz)
        if self._lib.fcsim_set_pid_gains(stage, axis, g) != 0:
            raise ValueError("ゲインが params_validate に通りません（負の値、i_limit > out_limit など）")

    def pid_gains(self, stage: int, axis: int) -> dict[str, float]:
        g = _F6()
        if self._lib.fcsim_get_pid_gains(stage, axis, g) != 0:
            raise ValueError("stage は 0/1、axis は 0..2")
        return dict(zip(("kp", "ki", "kd", "i_limit", "out_limit", "d_lpf_hz"), g[:]))

    def set_mixer_sign(self, sign) -> None:
        """行 M1..M4 × 列 roll,pitch,yaw の ±1 の表。"""
        flat = np.asarray(sign, dtype=np.int8).reshape(12)
        if self._lib.fcsim_set_mixer_sign(_I8x12(*flat.tolist())) != 0:
            raise ValueError("正しいX字の符号表ではありません（各列の和0・列同士が直交）")

    def mixer_sign(self) -> np.ndarray:
        buf = _I8x12()
        self._lib.fcsim_get_mixer_sign(buf)
        return np.array(buf[:], dtype=np.int8).reshape(4, 3)

    def set_param(self, name: str, value: float) -> None:
        if self._lib.fcsim_set_param(name.encode(), float(value)) != 0:
            raise ValueError(f"{name} = {value} は設定できません（名前が無いか、範囲外）")

    def param(self, name: str) -> float:
        v = C.c_float()
        if self._lib.fcsim_get_param(name.encode(), C.byref(v)) != 0:
            raise KeyError(name)
        return float(v.value)
