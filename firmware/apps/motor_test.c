/**
 * @file motor_test.c
 * @brief モーター単体試験: プロペラを外した状態で、モーターを1個ずつ低いデューティで回して確かめる。
 *
 * 目的: 配線（M1..M4 が基板どおりのモーターにつながっているか）、回転方向（設計どおりか）、
 *   電流、発熱を、安定化電源（4.0V、電流制限つき）から給電して確かめる。
 *   モーターの位置と設計上の回転方向（基板を上から見て）:
 *     M1=GP0  右後ろ 時計回り    M2=GP15 右前 反時計回り
 *     M3=GP18 左後ろ 反時計回り  M4=GP17 左前 時計回り
 *
 * ★安全★
 *   - 必ずプロペラを外す。給電は電流制限つきの安定化電源（4.0V）。
 *   - 同時に回るのは常に1個だけ（選んだモーター以外のデューティは必ず0）。
 *   - デューティ上限は MOTOR_TEST_DUTY_MAX（既定0.30。CMake の FC_MOTOR_TEST_DUTY_MAX で変える）。
 *   - 実際に出すデューティは目標へ毎秒0.5以下の速さで近づける（10msごとに0.005）。
 *   - 次のときは即座に全部止める（なだらかにはしない）: 停止コマンド、最後のキー入力から
 *     MOTOR_TEST_RUN_TIMEOUT_MS（既定10秒）、USBの切断。
 *   - ウォッチドッグ200ms。固まったら再起動し、motor_pwm_init で0に戻る。
 *     USB出力の詰まりで止まらないよう、printf の待ち時間の上限を20msにしてある（CMakeLists.txt）。
 *   - GP0 は SDK 既定のUART送信ピン。UART出力は無効にしてある（USBのみ）。
 *
 * コマンド（USBシリアルから1文字。Enter不要の端末を推奨）:
 *   1 2 3 4        モーターを選ぶ（選び直すと全部止めて目標を0に戻す）
 *   + / -          目標デューティを 0.05 ずつ上げ下げ（0 .. 上限）
 *   g              選んだモーターを回し始める（目標デューティへなだらかに）
 *   スペース 0 s    すぐに全部止める（実デューティも目標も0）
 *   b              電池電圧を1回表示
 *   h ?            ヘルプ
 * 回している間は1秒に1行表示する（止まっている間は表示しない）。
 *
 * 合格条件（詳細は docs/motor_test.md）:
 *   - 4個とも、選んだ番号のモーターだけが回る（他は回らない）。
 *   - 回転方向が設計どおり（M1,M4=時計回り、M2,M3=反時計回り）。
 *   - 停止すると電流が回す前の値に戻る。回している間の電流が電流制限に届かない。
 *   - 電池電圧が極端に下がらない。モーター・FETが熱くなりすぎない。
 *   - 10秒放置・USBケーブル抜きで止まる。
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "board_pins.h"
#include "battery.h"
#include "motor_pwm.h"
#include "params.h"

#ifndef MOTOR_TEST_DUTY_MAX
#define MOTOR_TEST_DUTY_MAX 0.30f   /* CMake からは数値（例 0.30）で渡る。使うときは (float) を付ける */
#endif
#ifndef MOTOR_TEST_RUN_TIMEOUT_MS
#define MOTOR_TEST_RUN_TIMEOUT_MS 10000u
#endif

#define WDT_TIMEOUT_MS     200u
#define TICK_US            10000u      /* 実デューティを更新する周期 */
#define RAMP_STEP          0.005f      /* 10msごと = 毎秒0.5 */
#define DUTY_STEP          0.05f       /* +/- 1回ぶん */
#define REPORT_PERIOD_US   1000000u
#define BATT_PERIOD_US     100000u
#define NO_BATT_BELOW_V    2.0f        /* これ未満なら電池なし（USB給電だけ）とみなす */

typedef struct {
    const char *pos;   /* 位置 */
    const char *dir;   /* 設計上の回転方向（基板を上から見て） */
} motor_info_t;

static const motor_info_t MOTORS[FC_NUM_MOTORS] = {
    { "右後ろ", "時計回り" },     /* M1 GP0  */
    { "右前",   "反時計回り" },   /* M2 GP15 */
    { "左後ろ", "反時計回り" },   /* M3 GP18 */
    { "左前",   "時計回り" },     /* M4 GP17 */
};

/* ピン割り当て（board_pins.h）と表の対応を確認（転記ミス防止） */
_Static_assert(BOARD_M1_GPIO == 0 && BOARD_M2_GPIO == 15 && BOARD_M3_GPIO == 18 && BOARD_M4_GPIO == 17,
               "モーターのGPIO割り当てが想定と違う");

static struct {
    int      sel;          /* 選んだモーター 0..3、未選択は -1 */
    int      steps;        /* 目標デューティ = DUTY_STEP × steps（整数で数え、誤差を溜めない） */
    int      steps_max;
    bool     running;
    float    duty;         /* 実際に出しているデューティ */
    uint32_t last_key_us;  /* 回している間の、最後のキー入力の時刻 */
} T = { -1, 0, 0, false, 0.0f, 0u };

static battery_state_t batt;
static params_t        prm;      /* 既定値（電池の換算に使う） */
static bool            no_batt;

static float target(void) { return DUTY_STEP * (float)T.steps; }

/**
 * ウォッチドッグに餌をやりながら待つ。WDTは200msなので、sleep_ms で200ms以上まとめて寝ると再起動してしまう。
 */
static void sleep_ms_fed(uint32_t ms)
{
    while (ms > 0u) {
        uint32_t d = ms > 50u ? 50u : ms;
        watchdog_update();
        sleep_ms(d);
        ms -= d;
    }
    watchdog_update();
}

static void print_help(void)
{
    printf("コマンド: 1-4=モーター選択 +/-=目標デューティ(0.05刻み) g=回す スペース/0/s=停止 b=電池電圧 h/?=ヘルプ\n");
    printf("  M1=右後ろ(時計回り) M2=右前(反時計回り) M3=左後ろ(反時計回り) M4=左前(時計回り)\n");
    printf("  上限 %.2f / %u秒キー入力なしで自動停止 / USB切断で停止 / 同時に回るのは1個だけ\n",
           (double)(float)MOTOR_TEST_DUTY_MAX, (unsigned)(MOTOR_TEST_RUN_TIMEOUT_MS / 1000u));
}

static void print_banner(void)
{
    printf("\n=== motor_test (モーター単体試験) ===\n");
    printf("!! プロペラを外すこと。電流制限つきの安定化電源(4.0V)で給電すること\n");
    printf("!! UART出力は無効（GP0はM1。USBシリアルだけ使う）\n");
    print_help();
}

/** 全部止める。実デューティも目標も0（選択は残す）。何度呼んでもよい。 */
static void stop_all(void)
{
    T.running = false;
    T.duty = 0.0f;
    T.steps = 0;
    motor_pwm_stop();
}

static void update_battery(uint32_t now_us)
{
    battery_update(&batt, &prm.battery, now_us / 1000u);
}

static void print_battery(void)
{
    if (no_batt) {
        printf(">> 電池電圧: %.2fV（2V未満。電池・電源が未接続の可能性）\n", (double)batt.raw_v);
    } else {
        printf(">> 電池電圧: %.2fV（ローパス後 %.2fV）\n", (double)batt.raw_v, (double)batt.voltage_v);
    }
}

static void print_status_line(uint32_t now_us)
{
    int32_t left_ms = (int32_t)MOTOR_TEST_RUN_TIMEOUT_MS - (int32_t)((uint32_t)(now_us - T.last_key_us) / 1000u);
    if (left_ms < 0) left_ms = 0;
    printf("M%d %s %s 目標=%.3f 実=%.3f Vbat=%.2fV 残り%.1fs\n",
           T.sel + 1, MOTORS[T.sel].pos, MOTORS[T.sel].dir,
           (double)target(), (double)T.duty, (double)batt.raw_v, (double)left_ms / 1000.0);
}

static void handle_key(int ch, uint32_t now_us)
{
    if (T.running) T.last_key_us = now_us;   /* 回している間は、どのキーでもタイムアウトを延ばす */

    switch (ch) {
    case '1': case '2': case '3': case '4':
        stop_all();
        T.sel = ch - '1';
        printf(">> M%d を選択（%s・設計の回転方向 %s）。全部止めて目標0。g で回す\n",
               T.sel + 1, MOTORS[T.sel].pos, MOTORS[T.sel].dir);
        break;
    case '+':
        if (T.steps < T.steps_max) T.steps++;
        printf(">> 目標デューティ %.2f%s\n", (double)target(), T.steps == T.steps_max ? "（上限）" : "");
        break;
    case '-':
        if (T.steps > 0) T.steps--;
        printf(">> 目標デューティ %.2f\n", (double)target());
        break;
    case 'g':
        if (T.sel < 0) {
            printf(">> 先に 1-4 でモーターを選ぶ\n");
        } else if (T.running) {
            printf(">> すでに回している（M%d）\n", T.sel + 1);
        } else {
            T.running = true;
            T.last_key_us = now_us;
            T.duty = 0.0f;
            printf(">> M%d 開始（目標 %.2f へなだらかに）%s\n", T.sel + 1, (double)target(),
                   T.steps == 0 ? " ※目標0。+ で上げる" : "");
        }
        break;
    case ' ': case '0': case 's':
        stop_all();
        printf(">> 停止（全部0）\n");
        break;
    case 'b':
        update_battery(now_us);
        print_battery();
        break;
    case 'h': case '?':
        print_help();
        break;
    default:
        break;
    }
}

/** 選んだモーターだけにデューティを出す。他は必ず0。 */
static void output_duty(void)
{
    float d[FC_NUM_MOTORS] = { 0.0f, 0.0f, 0.0f, 0.0f };
    if (T.running && T.sel >= 0 && T.sel < FC_NUM_MOTORS) d[T.sel] = T.duty;
    motor_pwm_set(d);
}

/** 実デューティを目標へ RAMP_STEP ずつ近づける（10msごとに1回呼ぶ）。 */
static void ramp_tick(void)
{
    float tgt = target();
    if (T.duty < tgt) {
        T.duty += RAMP_STEP;
        if (T.duty > tgt) T.duty = tgt;
    } else if (T.duty > tgt) {
        T.duty -= RAMP_STEP;
        if (T.duty < tgt) T.duty = tgt;
    }
}

int main(void)
{
    stdio_init_all();
    /* 最初に必ず全モーター0にする */
    motor_pwm_init();
    motor_pwm_stop();

    bool wdt_reboot = watchdog_caused_reboot();   /* enable の前に読む */
    watchdog_enable(WDT_TIMEOUT_MS, true);

    /* 上限（ステップ数）。浮動小数点の誤差で1つ減らないよう少し足してから切り捨てる */
    T.steps_max = (int)((float)MOTOR_TEST_DUTY_MAX / DUTY_STEP + 0.001f);

    params_set_defaults(&prm);
    battery_init(&batt);
    update_battery(time_us_32());
    no_batt = (batt.raw_v < NO_BATT_BELOW_V);

    while (!stdio_usb_connected()) {
        watchdog_update();
        sleep_ms(50);
    }
    sleep_ms_fed(500);

    print_banner();
    if (wdt_reboot) printf("!! ウォッチドッグで再起動した（モーターは全部0）\n");
    if (no_batt) printf("電池電圧 %.2fV（2V未満）。電源をつないでいるか確認\n", (double)batt.raw_v);
    else         printf("電池電圧 %.2fV\n", (double)batt.raw_v);

    bool     usb_was = true;
    uint32_t next_tick_us = time_us_32() + TICK_US;
    uint32_t last_report_us = time_us_32();
    uint32_t last_batt_us = last_report_us;

    for (;;) {
        watchdog_update();
        uint32_t now = time_us_32();

        /* USB切断: すぐ全部止める（毎回呼ぶ。冪等） */
        bool usb = stdio_usb_connected();
        if (!usb) {
            stop_all();
            usb_was = false;
            sleep_ms(5);
            continue;
        }
        if (!usb_was) {
            usb_was = true;
            sleep_ms_fed(500);
            printf("\n>> USB再接続。モーターは止まっている\n");
            print_help();
            next_tick_us = time_us_32() + TICK_US;
            continue;
        }

        /* キー入力（溜まっているぶんをすべて処理。停止キーが後ろに埋もれないように） */
        for (int n = 0; n < 16; n++) {
            int ch = getchar_timeout_us(0);
            if (ch == PICO_ERROR_TIMEOUT) break;
            handle_key(ch, now);
        }

        /* 自動停止 */
        if (T.running && (uint32_t)(now - T.last_key_us) >= MOTOR_TEST_RUN_TIMEOUT_MS * 1000u) {
            stop_all();
            printf(">> %u秒キー入力がないので自動停止（全部0）\n", (unsigned)(MOTOR_TEST_RUN_TIMEOUT_MS / 1000u));
        }

        /* 実デューティの更新と出力 */
        if ((int32_t)(now - next_tick_us) >= 0) {
            next_tick_us += TICK_US;
            if ((int32_t)(now - next_tick_us) >= 0) next_tick_us = now + TICK_US;   /* 大きく遅れたら追いかけない */
            if (T.running) {
                ramp_tick();
                output_duty();
            }
        }

        /* 電池電圧（ゆっくり更新） */
        if ((uint32_t)(now - last_batt_us) >= BATT_PERIOD_US) {
            last_batt_us = now;
            update_battery(now);
            no_batt = (batt.raw_v < NO_BATT_BELOW_V);
        }

        /* 表示: 回している間だけ、1秒に1行 */
        if ((uint32_t)(now - last_report_us) >= REPORT_PERIOD_US) {
            last_report_us = now;
            if (T.running) {
                watchdog_update();
                print_status_line(now);
                watchdog_update();
            }
        }

        sleep_ms(1);
    }
}
