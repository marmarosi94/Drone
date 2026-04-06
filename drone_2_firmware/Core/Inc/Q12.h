/*
 * Q16.h
 *
 *  Created on: Apr 2, 2026
 *      Author: balin
 */

#ifndef INC_Q12_H_
#define INC_Q12_H_

#include <stdint.h>
#include <stdlib.h>

typedef int32_t q12_t;

#define Q12_SHIFT 12
#define Q12_ONE   (1 << Q12_SHIFT)
#define Q12_HALF  (1 << (Q12_SHIFT - 1))

typedef struct
{
    q12_t x;
    q12_t y;
    q12_t z;
} vec3_q12_t;

typedef struct
{
    q12_t w;
    q12_t x;
    q12_t y;
    q12_t z;
} quat_q12_t;

//--------------------------------------------------
// Conversions
//--------------------------------------------------
static inline q12_t q12_from_float(float x)
{
    return (q12_t)(x * 4096.0f);
}

static inline q12_t q12_from_int(int32_t x)
{
    return x << Q12_SHIFT;
}

static inline int32_t q12_to_int(q12_t x)
{
    return x >> Q12_SHIFT;
}

//--------------------------------------------------
// Basic ops
//--------------------------------------------------
static inline q12_t q12_add(q12_t a, q12_t b)
{
    return a + b;
}

static inline q12_t q12_sub(q12_t a, q12_t b)
{
    return a - b;
}

static inline q12_t q12_mul(q12_t a, q12_t b)
{
    return (q12_t)(((int64_t)a * b) >> Q12_SHIFT);
}

static inline q12_t q12_div(q12_t a, q12_t b)
{
    return (q12_t)(((int64_t)a << Q12_SHIFT) / b);
}

static inline q12_t q12_abs(q12_t x)
{
    return x < 0 ? -x : x;
}

//--------------------------------------------------
// sqrt (Q12 correct scaling)
//--------------------------------------------------
static inline q12_t q12_sqrt(q12_t x)
{
    if (x <= 0) return 0;

    int64_t val = (int64_t)x << Q12_SHIFT;
    int64_t res = 0;
    int64_t bit = (int64_t)1 << 62;

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

    return (q12_t)res;
}

//--------------------------------------------------
// vec3
//--------------------------------------------------
static inline vec3_q12_t vec3_add(vec3_q12_t a, vec3_q12_t b)
{
    return (vec3_q12_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline vec3_q12_t vec3_sub(vec3_q12_t a, vec3_q12_t b)
{
    return (vec3_q12_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

// SAFE DOT (64-bit accumulator!)
static inline q12_t vec3_dot(vec3_q12_t a, vec3_q12_t b)
{
    int64_t sum =
        (int64_t)a.x * b.x +
        (int64_t)a.y * b.y +
        (int64_t)a.z * b.z;

    return (q12_t)(sum >> Q12_SHIFT);
}

static inline vec3_q12_t vec3_cross(vec3_q12_t a, vec3_q12_t b)
{
    vec3_q12_t r;

    r.x = q12_sub(q12_mul(a.y,b.z), q12_mul(a.z,b.y));
    r.y = q12_sub(q12_mul(a.z,b.x), q12_mul(a.x,b.z));
    r.z = q12_sub(q12_mul(a.x,b.y), q12_mul(a.y,b.x));

    return r;
}

// SAFE LENGTH
static inline q12_t vec3_length(vec3_q12_t v)
{
    int64_t sum =
        (int64_t)v.x * v.x +
        (int64_t)v.y * v.y +
        (int64_t)v.z * v.z;

    sum >>= Q12_SHIFT;

    return q12_sqrt((q12_t)sum);
}

static inline vec3_q12_t vec3_normalize(vec3_q12_t v)
{
    q12_t len = vec3_length(v);

    if(len == 0)
        return (vec3_q12_t){0,0,0};

    return (vec3_q12_t){
        q12_div(v.x, len),
        q12_div(v.y, len),
        q12_div(v.z, len)
    };
}

//--------------------------------------------------
// quaternion
//--------------------------------------------------
static inline int64_t sqrt_i64(int64_t n) {
    if (n < 0) return 0;
    if (n < 2) return n;

    int64_t x = n / 2 + 1; // Initial guess
    int64_t y = (x + n / x) / 2;

    while (y < x) {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}
static inline quat_q12_t quat_normalize(quat_q12_t q)
{
    // 1. Calculate sum in Q24 (int64 is necessary to prevent overflow)
    int64_t sum_q24 = (int64_t)q.w*q.w + (int64_t)q.x*q.x +
                      (int64_t)q.y*q.y + (int64_t)q.z*q.z;

    if(sum_q24 == 0) return q;

    // 2. Magnitude in Q12.
    // Since sum is Q24, sqrt(sum) naturally results in Q12.
    // Ensure your q12_sqrt function can handle a 64-bit input or
    // use a standard integer sqrt on the Q24 value.
    q12_t mag = (q12_t)sqrt_i64(sum_q24);

    if(mag == 0 || mag == 4096) return q; // Already normalized

    // 3. Division
    // Using q12_div is fine here, but scaling is key.
    q.w = q12_div(q.w, mag);
    q.x = q12_div(q.x, mag);
    q.y = q12_div(q.y, mag);
    q.z = q12_div(q.z, mag);

    return q;
}

static inline quat_q12_t quat_mul(quat_q12_t q, quat_q12_t p)
{
    quat_q12_t r;
    // Use 64-bit accumulators to prevent overflow before the final shift
    int64_t w, x, y, z;

    w = ((int64_t)q.w * p.w) - ((int64_t)q.x * p.x) - ((int64_t)q.y * p.y) - ((int64_t)q.z * p.z);
    x = ((int64_t)q.w * p.x) + ((int64_t)q.x * p.w) + ((int64_t)q.y * p.z) - ((int64_t)q.z * p.y);
    y = ((int64_t)q.w * p.y) - ((int64_t)q.x * p.z) + ((int64_t)q.y * p.w) + ((int64_t)q.z * p.x);
    z = ((int64_t)q.w * p.z) + ((int64_t)q.x * p.y) - ((int64_t)q.y * p.x) + ((int64_t)q.z * p.w);

    // Shift once to return to Q12
    r.w = (q12_t)(w >> 12);
    r.x = (q12_t)(x >> 12);
    r.y = (q12_t)(y >> 12);
    r.z = (q12_t)(z >> 12);

    return r;
}

static inline quat_q12_t quat_conjugate(quat_q12_t q)
{
    return (quat_q12_t){q.w, -q.x, -q.y, -q.z};
}

static inline q12_t quat_norm_sq(quat_q12_t q)
{
    int64_t sum =
        (int64_t)q.w*q.w +
        (int64_t)q.x*q.x +
        (int64_t)q.y*q.y +
        (int64_t)q.z*q.z;

    return (q12_t)(sum >> Q12_SHIFT);
}

static inline quat_q12_t quat_inverse(quat_q12_t q)
{
    quat_q12_t r = quat_conjugate(q);
    q12_t norm = quat_norm_sq(q);

    r.w = q12_div(r.w, norm);
    r.x = q12_div(r.x, norm);
    r.y = q12_div(r.y, norm);
    r.z = q12_div(r.z, norm);

    return r;
}

static inline quat_q12_t quat_div(quat_q12_t q, quat_q12_t p)
{
    return quat_mul(q, quat_inverse(p));
}

#endif
