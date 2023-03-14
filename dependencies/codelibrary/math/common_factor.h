//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_COMMON_FACTOR_H_
#define CODELIBRARY_MATH_COMMON_FACTOR_H_

#include <algorithm>
#include <cmath>
#include <limits>

#include "codelibrary/math/number/bigint.h"

namespace cl {

/**
 * Compute greatest common factor of two non-negative integers by Mixed Binary
 * Euclid Algorithm.
 *
 * It is faster than BinaryGCD.
 *
 * Reference:
 *   Sedjelmaci S M. The Mixed Binary Euclid Algorithm[J]. Electronic Notes in
 *   Discrete Mathematics, 2009, 35:169-176.
 */
template <typename T>
T MixedBinaryGCD(T u, T v) {
    CHECK(u >= 0 && v >= 0);

    if (u == 0) return v;
    if (v == 0) return u;

    if (u < v) std::swap(u, v);

    int c1 = 0, c2 = 0;
    while (u % 2 == 0) {
        u >>= 1;
        ++c1;
    }
    while (v % 2 == 0) {
        v >>= 1;
        ++c2;
    }
    int shifts = std::min(c1, c2);

    while (v > 1) {
        u %= v;
        v -= u;
        if (u == 0) return v << shifts;
        if (v == 0) return u << shifts;

        while (u % 2 == 0) u >>= 1;
        while (v % 2 == 0) v >>= 1;

        if (u < v) std::swap(u, v);
    }
    return (v == 1 ? v : u) << shifts;
}

/**
 * Compute greatest common factor by Stein's method.
 */
template <typename T>
T BinaryGCD(T u, T v) {
    CHECK(u >= 0 && v >= 0);

    if (!u) return v;
    if (!v) return u;

    int c1 = 0, c2 = 0;
    while (u % 2 == 0) {
        u >>= 1;
        ++c1;
    }
    while (v % 2 == 0) {
        v >>= 1;
        ++c2;
    }

    while (u != v) {
        if (v > u) std::swap(u, v);
        u -= v;
        while (u % 2 == 0) u >>= 1;
    }

    return u << std::min(c1, c2);
}

/**
 * Compute greatest common factor by Euclidean algorithm.
 *
 * Faster than MixedBinaryGCD when the number is small.
 */
template <typename T>
T EuclideanGCD(T u, T v) {
    CHECK(u >= 0 && v >= 0);

    while (v != 0) {
        u %= v;
        std::swap(u, v);
    }
    return u;
}

/**
 * Greatest common factor of two positive integer u and v.
 */
template <typename T>
T GCD(T u, T v) {
    return MixedBinaryGCD(u, v);
}

/**
 * For small integers, Euclidean GCD is faster.
 */
inline uint32_t GCD(uint32_t u, uint32_t v) {
    return EuclideanGCD(u, v);
}
inline int32_t GCD(int32_t u, int32_t v)    {
    return EuclideanGCD(u, v);
}

/**
 * The extended GCD algorithm is an extension to the Euclidean GCD algorithm.
 * Besides finding the greatest common divisor of integers a and b,
 * as the Euclidean algorithm does, it also finds integers x and y
 * (one of which is typically negative) that satisfy Bezout's identity:
 *      a * x + b * y = GCD(a, b).
 * The extended Euclidean algorithm is particularly useful when a and b are
 * co-prime, since x is the multiplicative inverse of a modulo b,
 * and y is the multiplicative inverse of b modulo a.
 *
 * Note that, x and y must be the signed integers.
 */
template <typename T>
T ExtendedGCD(const T& a, const T& b, T* x, T* y) {
    CHECK(a >= 0 && b >= 0);
    CHECK(x != 0 && y != 0);

    T x0 = 1, y0 = 0;
    T x1 = 0, y1 = 1;
    T c = a, d = b;
    while (d != 0) {
        T k = c / d;
        T r = c % d;

        T t = c;
        c = d;
        d = r;

        t = x0;
        x0 = x1;
        x1 = t - k * x1;

        t = y0;
        y0 = y1;
        y1 = t - k * y1;
    }

    *x = x0, *y = y0;
    return c;
}

} // namespace cl

#endif // CODELIBRARY_MATH_COMMON_FACTOR_H_
