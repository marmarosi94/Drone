/*
 * Q16.h
 *
 * Created on: Apr 7, 2026
 * Author: Gemini
 */

#ifndef INC_Q16_H_
#define INC_Q16_H_

#include <stdint.h>
#include <stdlib.h>

typedef int32_t q16_t;

#define Q16_SHIFT 16
#define Q16_ONE   (1 << Q16_SHIFT)
#define Q16_HALF  (1 << (Q16_SHIFT - 1))

typedef struct
{
    q16_t x;
    q16_t y;
    q16_t z;
} vec3_q16_t;

typedef struct
{
    q16_t w;
    q16_t x;
    q16_t y;
    q16_t z;
} quat_q16_t;

//--------------------------------------------------
// Conversions
//--------------------------------------------------
static inline q16_t q16_from_float(float x)
{
    return (q16_t)(x * 65536.0f);
}

static inline float q16_to_float(q16_t x)
{
    return (float)x / 65536.0f;
}

static inline q16_t q16_from_int(int32_t x)
{
    return x << Q16_SHIFT;
}

static inline int32_t q16_to_int(q16_t x)
{
    return x >> Q16_SHIFT;
}

//--------------------------------------------------
// Basic ops
//--------------------------------------------------
static inline q16_t q16_add(q16_t a, q16_t b)
{
    return a + b;
}

static inline q16_t q16_sub(q16_t a, q16_t b)
{
    return a - b;
}

static inline q16_t q16_mul(q16_t a, q16_t b)
{
    return (q16_t)(((int64_t)a * b) >> Q16_SHIFT);
}

static inline q16_t q16_div(q16_t a, q16_t b)
{
    if (b == 0) return 0;
    return (q16_t)(((int64_t)a << Q16_SHIFT) / b);
}

static inline q16_t q16_abs(q16_t x)
{
    return x < 0 ? -x : x;
}

//--------------------------------------------------
// Sqrt (Q16 correct scaling)
//--------------------------------------------------
static inline q16_t q16_sqrt(q16_t x)
{
    if (x <= 0) return 0;

    // To get a Q16 result from a Q16 input,
    // we need to sqrt(x * 2^16), which is equivalent to sqrt(x) * 2^8.
    // However, for maximum precision, we scale to Q32 internally:
    int64_t val = (int64_t)x << Q16_SHIFT;
    int64_t res = 0;
    int64_t bit = (int64_t)1 << 62; // Must be even power of 2

    while (bit > val)
        bit >>= 2;

    while (bit != 0)
    {
        if (val >= res + bit)
        {
            val -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }

    return (q16_t)res;
}

//--------------------------------------------------
// vec3
//--------------------------------------------------
static inline vec3_q16_t vec3_add(vec3_q16_t a, vec3_q16_t b)
{
    return (vec3_q16_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline vec3_q16_t vec3_sub(vec3_q16_t a, vec3_q16_t b)
{
    return (vec3_q16_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline q16_t vec3_dot(vec3_q16_t a, vec3_q16_t b)
{
    int64_t sum =
        (int64_t)a.x * b.x +
        (int64_t)a.y * b.y +
        (int64_t)a.z * b.z;

    return (q16_t)(sum >> Q16_SHIFT);
}

static inline vec3_q16_t vec3_cross(vec3_q16_t a, vec3_q16_t b)
{
    vec3_q16_t r;
    r.x = q16_sub(q16_mul(a.y, b.z), q16_mul(a.z, b.y));
    r.y = q16_sub(q16_mul(a.z, b.x), q16_mul(a.x, b.z));
    r.z = q16_sub(q16_mul(a.x, b.y), q16_mul(a.y, b.x));
    return r;
}

static inline q16_t vec3_length(vec3_q16_t v)
{
    int64_t sum_q32 = (int64_t)v.x * v.x + (int64_t)v.y * v.y + (int64_t)v.z * v.z;
    // sum_q32 is in Q32. Sqrt of Q32 results in Q16.
    // We use a custom 64-bit sqrt to maintain precision before the final cast.

    if (sum_q32 <= 0) return 0;

    int64_t res = 0;
    int64_t bit = (int64_t)1 << 62;
    while (bit > sum_q32) bit >>= 2;
    while (bit != 0) {
        if (sum_q32 >= res + bit) {
            sum_q32 -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return (q16_t)res;
}

static inline vec3_q16_t vec3_normalize(vec3_q16_t v)
{
    q16_t len = vec3_length(v);
    if(len == 0) return (vec3_q16_t){0,0,0};

    return (vec3_q16_t){
        q16_div(v.x, len),
        q16_div(v.y, len),
        q16_div(v.z, len)
    };
}

//--------------------------------------------------
// quaternion
//--------------------------------------------------
static inline quat_q16_t quat_normalize(quat_q16_t q)
{
    int64_t sum_q32 = (int64_t)q.w*q.w + (int64_t)q.x*q.x +
                      (int64_t)q.y*q.y + (int64_t)q.z*q.z;

    if(sum_q32 == 0) return q;

    // Square root of sum (Q32) gives magnitude in Q16
    int64_t res = 0;
    int64_t bit = (int64_t)1 << 62;
    while (bit > sum_q32) bit >>= 2;
    while (bit != 0) {
        if (sum_q32 >= res + bit) {
            sum_q32 -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }

    q16_t mag = (q16_t)res;
    if(mag == 0 || mag == Q16_ONE) return q;

    q.w = q16_div(q.w, mag);
    q.x = q16_div(q.x, mag);
    q.y = q16_div(q.y, mag);
    q.z = q16_div(q.z, mag);

    return q;
}

static inline quat_q16_t quat_mul(quat_q16_t q, quat_q16_t p)
{
    quat_q16_t r;
    int64_t w, x, y, z;

    w = ((int64_t)q.w * p.w) - ((int64_t)q.x * p.x) - ((int64_t)q.y * p.y) - ((int64_t)q.z * p.z);
    x = ((int64_t)q.w * p.x) + ((int64_t)q.x * p.w) + ((int64_t)q.y * p.z) - ((int64_t)q.z * p.y);
    y = ((int64_t)q.w * p.y) - ((int64_t)q.x * p.z) + ((int64_t)q.y * p.w) + ((int64_t)q.z * p.x);
    z = ((int64_t)q.w * p.z) + ((int64_t)q.x * p.y) - ((int64_t)q.y * p.x) + ((int64_t)q.z * p.w);

    r.w = (q16_t)(w >> Q16_SHIFT);
    r.x = (q16_t)(x >> Q16_SHIFT);
    r.y = (q16_t)(y >> Q16_SHIFT);
    r.z = (q16_t)(z >> Q16_SHIFT);

    return r;
}

static inline quat_q16_t quat_conjugate(quat_q16_t q)
{
    return (quat_q16_t){q.w, -q.x, -q.y, -q.z};
}

static inline q16_t quat_norm_sq(quat_q16_t q)
{
    int64_t sum =
        (int64_t)q.w*q.w +
        (int64_t)q.x*q.x +
        (int64_t)q.y*q.y +
        (int64_t)q.z*q.z;

    return (q16_t)(sum >> Q16_SHIFT);
}

static inline quat_q16_t quat_inverse(quat_q16_t q)
{
    quat_q16_t r = quat_conjugate(q);
    q16_t norm = quat_norm_sq(q);

    if (norm == 0) return r; // Avoid division by zero

    r.w = q16_div(r.w, norm);
    r.x = q16_div(r.x, norm);
    r.y = q16_div(r.y, norm);
    r.z = q16_div(r.z, norm);

    return r;
}

static inline quat_q16_t quat_div(quat_q16_t q, quat_q16_t p)
{
    return quat_mul(q, quat_inverse(p));
}

#endif /* INC_Q16_H_ */
