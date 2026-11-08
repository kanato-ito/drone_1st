"""初代C 確認用シミュレータ（設計メモ 6章「シミュレータ」）。

fc      … ファーム（libfcsim）を ctypes で呼ぶ
model   … 機体モデルの差し込み口（DroneModel）と、物理なしの試験台モデル（BenchModel）
runner  … 時間の進め方（物理の1刻み = IMUの1サンプル、8サンプルごとにファームの1周期）
plot    … 記録のグラフ
"""
from .fc import FlightComputer, PID_ANGLE, PID_RATE, ROLL, PITCH, YAW  # noqa: F401
from .model import BenchModel, Disturbance, DroneModel, NO_DISTURBANCE  # noqa: F401
from .runner import Controls, Log, Scenario, Simulation, SAMPLES_PER_CYCLE  # noqa: F401
