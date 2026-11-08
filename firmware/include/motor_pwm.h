/**
 * @file motor_pwm.h
 * @brief モーターPWM出力（20kHz）。M1/M4はスライス0を共有、M2=スライス7B、M3=スライス1A。
 *
 * ポート（差し替え境界）: 実機は port_pico ディレクトリの motor_pwm.c、シミュレータは同じ関数を自前の.cで実装する。
 * 偽物は motor_pwm_set で受けたデューティ(0..1、範囲外/NaNは0)を覚え、motor_pwm_stop で全部0にする。
 *
 * コア0だけが使う。SDK非依存（PWMは .c の中）。
 * 150MHz÷20kHz = 7500カウントが目安（分解能約13bit。.c 実装時に確認）。
 */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "fc_types.h"

/** PWMを初期化（出力0%）。コア0、起動時に1回。ピンは board_pins.h。 */
void motor_pwm_init(void);

/** 4モーターのデューティ(0..1)を設定。範囲外/NaNは0に丸める。コア0、1周期に1回（ミキサー直後）。 */
void motor_pwm_set(const float duty[FC_NUM_MOTORS]);

/** 全モーター0%。非常停止・アーム解除・タイムアウトで呼ぶ。コア0、必要なとき（冪等）。 */
void motor_pwm_stop(void);

#endif /* MOTOR_PWM_H */
