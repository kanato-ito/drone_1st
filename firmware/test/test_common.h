/** @file test_common.h  試験の共通部品（CHECK マクロと件数）。各 test_*.c が使う。 */
#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

extern int g_checks, g_fail, g_test_checks;

#define CHECK(cond) do { g_checks++; g_test_checks++; if (!(cond)) { g_fail++; \
    printf("    FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { float a_ = (float)(a), b_ = (float)(b), t_ = (float)(tol); \
    g_checks++; g_test_checks++; \
    if (!(fabsf(a_ - b_) <= t_)) { g_fail++; \
    printf("    FAIL %s:%d  %s = %g, 期待 %g ± %g\n", __FILE__, __LINE__, #a, (double)a_, (double)b_, (double)t_); } } while (0)

#define DEG 0.017453293f

typedef struct {
    const char *name;
    void (*fn)(void);
} test_t;

/* 各ファイルの試験の表 */
extern const test_t g_tests_fsm[];       extern const int g_tests_fsm_n;
extern const test_t g_tests_guard[];     extern const int g_tests_guard_n;
extern const test_t g_tests_descent[];   extern const int g_tests_descent_n;
extern const test_t g_tests_ipc[];       extern const int g_tests_ipc_n;
extern const test_t g_tests_loop[];      extern const int g_tests_loop_n;

#endif /* TEST_COMMON_H */
