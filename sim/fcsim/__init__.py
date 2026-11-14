"""初代C 確認用シミュレータ（設計メモ 6章「シミュレータ」）。

fc      … ファーム（libfcsim）を ctypes で呼ぶ
model   … 機体モデルの差し込み口（DroneModel）と、物理なしの試験台モデル（BenchModel）
drone   … 機体モデル（モーター・電池・剛体・床・IMU）
runner  … 時間の進め方（物理の1刻み = IMUの1サンプル、8サンプルごとにファームの1周期）
plot    … 記録のグラフ
"""
from .fc import FlightComputer, PID_ANGLE, PID_RATE, ROLL, PITCH, YAW  # noqa: F401
from .model import BenchModel, Disturbance, DroneModel, NO_DISTURBANCE  # noqa: F401
from .runner import Controls, Log, Scenario, Simulation, SAMPLES_PER_CYCLE, MOUNT_BODY_TO_BOARD  # noqa: F401
from .drone import Drone, DroneParams  # noqa: F401,E402
