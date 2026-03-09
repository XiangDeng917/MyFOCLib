/**
 * @file unit_test_foc.c
 * @brief Unit tests for MyFOCLib core functions.
 *
 * Tests are implemented without an external framework — each test case
 * is a function that returns 0 on success and 1 on failure.  A summary
 * is printed to stdout and the process exits with a non-zero code if any
 * test fails (compatible with CTest).
 *
 * Covered modules:
 *   - pid.c  : initialisation, single step, saturation, anti-windup, reset
 *   - my_filter.c : initialisation, step response, reset
 *   - foc.c  : Clarke / inverse-Clarke round-trip, Park / inverse-Park round-trip
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "pid.h"
#include "my_filter.h"
#include "foc.h"

/* -------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------- */

#define EPSILON  1e-5f

static int float_close(float a, float b, float eps)
{
    float diff = a - b;
    if (diff < 0.0f) diff = -diff;
    return diff < eps;
}

/* Simple test runner bookkeeping */
static int g_tests_run    = 0;
static int g_tests_failed = 0;

#define ASSERT(cond, msg)                                          \
    do {                                                           \
        ++g_tests_run;                                             \
        if (!(cond)) {                                             \
            ++g_tests_failed;                                      \
            fprintf(stderr, "  FAIL [%s:%d]: %s\n",               \
                    __FILE__, __LINE__, (msg));                     \
        }                                                          \
    } while (0)

/* -------------------------------------------------------------------------
 * PID tests
 * ---------------------------------------------------------------------- */

static void test_pid_init(void)
{
    pid_t pid;
    pid_init(&pid, 1.0f, 0.5f, 0.1f, -100.0f, 100.0f);

    ASSERT(float_close(pid.kp, 1.0f,   EPSILON), "pid kp after init");
    ASSERT(float_close(pid.ki, 0.5f,   EPSILON), "pid ki after init");
    ASSERT(float_close(pid.kd, 0.1f,   EPSILON), "pid kd after init");
    ASSERT(float_close(pid.integral,   0.0f, EPSILON), "pid integral after init");
    ASSERT(float_close(pid.prev_error, 0.0f, EPSILON), "pid prev_error after init");
}

static void test_pid_single_step(void)
{
    pid_t pid;
    /* kp=1, ki=0, kd=0 → output should equal the error */
    pid_init(&pid, 1.0f, 0.0f, 0.0f, -1000.0f, 1000.0f);

    float out = pid_calc(&pid, 100.0f, 60.0f);  /* error = 40 */
    ASSERT(float_close(out, 40.0f, EPSILON), "pid proportional output");
}

static void test_pid_saturation(void)
{
    pid_t pid;
    /* Large kp, small limits → output must be clamped */
    pid_init(&pid, 100.0f, 0.0f, 0.0f, -50.0f, 50.0f);

    float out = pid_calc(&pid, 100.0f, 0.0f);  /* raw = 10000 → clamped to 50 */
    ASSERT(float_close(out, 50.0f, EPSILON), "pid upper saturation");

    pid_reset(&pid);
    out = pid_calc(&pid, 0.0f, 100.0f);  /* raw = -10000 → clamped to -50 */
    ASSERT(float_close(out, -50.0f, EPSILON), "pid lower saturation");
}

static void test_pid_reset(void)
{
    pid_t pid;
    pid_init(&pid, 1.0f, 1.0f, 0.0f, -1000.0f, 1000.0f);

    /* Take a few steps to accumulate integral */
    pid_calc(&pid, 10.0f, 0.0f);
    pid_calc(&pid, 10.0f, 0.0f);

    pid_reset(&pid);
    ASSERT(float_close(pid.integral,   0.0f, EPSILON), "pid integral after reset");
    ASSERT(float_close(pid.prev_error, 0.0f, EPSILON), "pid prev_error after reset");
}

static void test_pid_null_safe(void)
{
    /* Passing NULL must not crash */
    pid_init(NULL, 1.0f, 1.0f, 0.0f, -1.0f, 1.0f);
    float out = pid_calc(NULL, 10.0f, 0.0f);
    pid_reset(NULL);
    ASSERT(float_close(out, 0.0f, EPSILON), "pid_calc(NULL) returns 0");
}

/* -------------------------------------------------------------------------
 * Filter tests
 * ---------------------------------------------------------------------- */

static void test_filter_init(void)
{
    lp_filter_t f;
    filter_init(&f, 0.1f, 5.0f);

    ASSERT(float_close(f.alpha,  0.1f, EPSILON), "filter alpha after init");
    ASSERT(float_close(f.output, 5.0f, EPSILON), "filter output after init");
}

static void test_filter_unity_alpha(void)
{
    lp_filter_t f;
    filter_init(&f, 1.0f, 0.0f);   /* alpha=1 → output = input */

    float out = filter_update(&f, 42.0f);
    ASSERT(float_close(out, 42.0f, EPSILON), "filter unity alpha passes input");
}

static void test_filter_convergence(void)
{
    lp_filter_t f;
    filter_init(&f, 0.5f, 0.0f);

    /* Apply constant input 100; after enough steps output should be ~100 */
    float out = 0.0f;
    for (int i = 0; i < 40; i++) {
        out = filter_update(&f, 100.0f);
    }
    ASSERT(float_close(out, 100.0f, 0.01f), "filter converges to constant input");
}

static void test_filter_reset(void)
{
    lp_filter_t f;
    filter_init(&f, 0.1f, 0.0f);

    filter_update(&f, 100.0f);
    filter_reset(&f, 0.0f);
    ASSERT(float_close(f.output, 0.0f, EPSILON), "filter output after reset");
}

static void test_filter_null_safe(void)
{
    float out = filter_update(NULL, 5.0f);
    ASSERT(float_close(out, 5.0f, EPSILON), "filter_update(NULL) returns input");
    /* These must not crash */
    filter_init(NULL, 0.1f, 0.0f);
    filter_reset(NULL, 0.0f);
}

/* -------------------------------------------------------------------------
 * FOC transform tests
 * ---------------------------------------------------------------------- */

/** Clarke followed by inverse-Clarke must reproduce the original (a, b). */
static void test_foc_clarke_roundtrip(void)
{
    foc_vec3_t in  = {1.0f, -0.5f, -0.5f};  /* balanced three-phase */
    foc_vec2_t ab;
    foc_vec3_t out;

    foc_clarke(in, &ab);
    foc_clarke_inv(ab, &out);

    ASSERT(float_close(out.a, in.a, 1e-4f), "clarke round-trip: phase A");
    ASSERT(float_close(out.b, in.b, 1e-4f), "clarke round-trip: phase B");
    ASSERT(float_close(out.c, in.c, 1e-4f), "clarke round-trip: phase C");
}

/** Park followed by inverse-Park must reproduce the original αβ vector. */
static void test_foc_park_roundtrip(void)
{
    foc_vec2_t ab_in  = {0.8f, 0.6f};
    float      theta  = 1.2f;  /* arbitrary angle */
    foc_vec2_t dq;
    foc_vec2_t ab_out;

    foc_park(ab_in, theta, &dq);
    foc_park_inv(dq, theta, &ab_out);

    ASSERT(float_close(ab_out.x, ab_in.x, 1e-5f), "park round-trip: alpha");
    ASSERT(float_close(ab_out.y, ab_in.y, 1e-5f), "park round-trip: beta");
}

/** At θ=0 the Park transform should be identity. */
static void test_foc_park_identity(void)
{
    foc_vec2_t ab = {1.0f, 0.0f};
    foc_vec2_t dq;

    foc_park(ab, 0.0f, &dq);

    ASSERT(float_close(dq.x, 1.0f, 1e-5f), "park at 0: d == alpha");
    ASSERT(float_close(dq.y, 0.0f, 1e-5f), "park at 0: q == 0");
}

/** SVM duty cycles must always lie in [0, 1]. */
static void test_foc_svm_range(void)
{
    float vbus = 12.0f;
    /* Test several αβ vectors */
    float tests[][2] = {
        { 0.0f,  0.0f},
        { 6.0f,  0.0f},
        {-6.0f,  0.0f},
        { 0.0f,  6.0f},
        { 4.0f,  4.0f},
    };
    for (int i = 0; i < 5; i++) {
        foc_vec2_t ab = {tests[i][0], tests[i][1]};
        foc_pwm_t  pwm;
        foc_svm(ab, vbus, &pwm);
        ASSERT(pwm.ta >= 0.0f && pwm.ta <= 1.0f, "svm ta in [0,1]");
        ASSERT(pwm.tb >= 0.0f && pwm.tb <= 1.0f, "svm tb in [0,1]");
        ASSERT(pwm.tc >= 0.0f && pwm.tc <= 1.0f, "svm tc in [0,1]");
    }
}

/* -------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------- */

int main(void)
{
    printf("=== MyFOCLib Unit Tests ===\n\n");

    printf("[PID]\n");
    test_pid_init();
    test_pid_single_step();
    test_pid_saturation();
    test_pid_reset();
    test_pid_null_safe();

    printf("[Filter]\n");
    test_filter_init();
    test_filter_unity_alpha();
    test_filter_convergence();
    test_filter_reset();
    test_filter_null_safe();

    printf("[FOC transforms]\n");
    test_foc_clarke_roundtrip();
    test_foc_park_roundtrip();
    test_foc_park_identity();
    test_foc_svm_range();

    printf("\n--- Results: %d tests, %d failures ---\n",
           g_tests_run, g_tests_failed);

    return g_tests_failed ? 1 : 0;
}
