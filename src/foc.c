/**
 * @file foc.c
 * @brief FOC coordinate transforms and space-vector modulation.
 *
 * Reference: "Field Oriented Control of Permanent Magnet Synchronous Motors"
 * (TI Application Report SPRABQ8)
 */

#include "foc.h"
#include <math.h>

/* -------------------------------------------------------------------------
 * Coordinate transformations
 * ---------------------------------------------------------------------- */

void foc_clarke(foc_vec3_t abc, foc_vec2_t *ab)
{
    if (!ab) return;

    /* Power-invariant Clarke (balanced three-phase, ia+ib+ic=0):
     *   alpha = ia
     *   beta  = (ia + 2*ib) / sqrt(3)
     */
    ab->x = abc.a;
    ab->y = (abc.a + 2.0f * abc.b) * 0.57735026919f; /* 1/sqrt(3) */
}

void foc_clarke_inv(foc_vec2_t ab, foc_vec3_t *abc)
{
    if (!abc) return;

    /* Inverse Clarke:
     *   Va =  alpha
     *   Vb = -alpha/2 + beta*sqrt(3)/2
     *   Vc = -alpha/2 - beta*sqrt(3)/2
     */
    abc->a =  ab.x;
    abc->b = -ab.x * 0.5f + ab.y * 0.86602540378f; /* sqrt(3)/2 */
    abc->c = -ab.x * 0.5f - ab.y * 0.86602540378f;
}

void foc_park(foc_vec2_t ab, float theta, foc_vec2_t *dq)
{
    if (!dq) return;

    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    dq->x =  ab.x * cos_t + ab.y * sin_t;  /* d */
    dq->y = -ab.x * sin_t + ab.y * cos_t;  /* q */
}

void foc_park_inv(foc_vec2_t dq, float theta, foc_vec2_t *ab)
{
    if (!ab) return;

    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    ab->x = dq.x * cos_t - dq.y * sin_t;  /* alpha */
    ab->y = dq.x * sin_t + dq.y * cos_t;  /* beta  */
}

/* -------------------------------------------------------------------------
 * Space-vector modulation
 * ---------------------------------------------------------------------- */

void foc_svm(foc_vec2_t ab, float vbus, foc_pwm_t *pwm)
{
    if (!pwm || vbus <= 0.0f) return;

    /* Inverse Clarke to get three-phase references */
    foc_vec3_t ref;
    foc_clarke_inv(ab, &ref);

    /* Normalise to [0, 1] duty cycle */
    float inv_vbus = 1.0f / vbus;
    float da = ref.a * inv_vbus * 0.5f + 0.5f;
    float db = ref.b * inv_vbus * 0.5f + 0.5f;
    float dc = ref.c * inv_vbus * 0.5f + 0.5f;

    /* Clamp to valid range */
    pwm->ta = da < 0.0f ? 0.0f : (da > 1.0f ? 1.0f : da);
    pwm->tb = db < 0.0f ? 0.0f : (db > 1.0f ? 1.0f : db);
    pwm->tc = dc < 0.0f ? 0.0f : (dc > 1.0f ? 1.0f : dc);
}
