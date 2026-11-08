"""記録（runner.Log）をグラフにする。画面が無い環境でも PNG に書けるよう Agg を使う。"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

RAD2DEG = 180.0 / np.pi
AXES = ("roll", "pitch", "yaw")
MOTORS = ("M1", "M2", "M3", "M4")


def plot_log(log, path: str | Path, title: str = "") -> Path:
    t = log["t"]
    fig, ax = plt.subplots(5, 1, figsize=(11, 13), sharex=True)

    # 1. 姿勢（推定と目標、機体モデルが本当の値を出していればそれも）
    for i, name in enumerate(AXES):
        line, = ax[0].plot(t, log["angle_rad"][:, i] * RAD2DEG, label=f"{name} 推定")
        ax[0].plot(t, log["target_rad"][:, i] * RAD2DEG, "--", color=line.get_color(), alpha=0.6,
                   label=f"{name} 目標")
        if "model_true_angle_rad" in log:
            ax[0].plot(t, log["model_true_angle_rad"][:, i] * RAD2DEG, ":", color=line.get_color(),
                       label=f"{name} 本当")
    ax[0].set_ylabel("姿勢 [deg]")

    # 2. 角速度（推定と、外側PIDが出した目標角速度）
    for i, name in enumerate(AXES):
        line, = ax[1].plot(t, log["rate_rps"][:, i] * RAD2DEG, label=f"{name} 推定")
        ax[1].plot(t, log["rate_target_rps"][:, i] * RAD2DEG, "--", color=line.get_color(), alpha=0.6,
                   label=f"{name} 目標")
    ax[1].set_ylabel("角速度 [deg/s]")

    # 3. 制御器の出力と内側PIDの積分
    for i, name in enumerate(AXES):
        line, = ax[2].plot(t, log["out_torque_g"][:, i], label=f"{name} 指令")
        ax[2].plot(t, log["rate_integ_g"][:, i], ":", color=line.get_color(), label=f"{name} I項")
    ax[2].set_ylabel("指令 [g]")

    # 4. デューティ
    for i, name in enumerate(MOTORS):
        ax[3].plot(t, log["duty"][:, i], label=name)
    ax[3].set_ylabel("デューティ")

    # 5. 電池電圧（ファームのローパス後と、機体モデルの端子電圧）
    ax[4].plot(t, log["battery_v_true"], label="端子電圧（モデル）", alpha=0.6)
    ax[4].plot(t, log["batt_v"], label="ファームのローパス後")
    ax[4].set_ylabel("電池 [V]")
    ax[4].ticklabel_format(axis="y", useOffset=False)
    ax[4].set_xlabel("時間 [s]")

    armed = log["armed"] > 0.5
    for a in ax:
        if armed.any():
            a.axvspan(t[armed][0], t[-1], color="0.9", zorder=0)
        a.grid(True, alpha=0.3)
        a.legend(loc="upper right", fontsize=7, ncol=3)
    if title:
        fig.suptitle(title + "（灰色＝アーム中）")
    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def _setup_font() -> None:
    """日本語の凡例が豆腐にならないよう、入っている日本語フォントを探して使う（無ければそのまま）。"""
    from matplotlib import font_manager

    for name in ("Hiragino Sans", "Hiragino Kaku Gothic ProN", "Noto Sans CJK JP", "IPAexGothic",
                 "Yu Gothic", "Meiryo", "TakaoGothic"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            matplotlib.rcParams["font.family"] = name
            return
    # 日本語フォントが無い環境（クラウドなど）: 文字は□になるが、グラフの線は正しい。警告だけ止める
    import warnings

    warnings.filterwarnings("ignore", message="Glyph .* missing from font")


_setup_font()
