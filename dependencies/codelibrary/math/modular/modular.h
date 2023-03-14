//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MODULAR_MODULAR_H_
#define CODELIBRARY_MATH_MODULAR_MODULAR_H_

#include <cstdint>

#include "codelibrary/math/common_factor.h"
#include "codelibrary/math/number/bigint.h"

namespace cl {

/**
 * Compute a % n, ensure the answer is between [0, n).
 *
 * Require n > 0.
 */
template <typename T>
T Mod(const T& a, const T& n) {
    return a < 0 ? a % n + n : (a >= n ? a % n : a);
}

/**
 * Compute (a + b) % n.
 *
 * Require 0 <= a < n, 0 <= b < n and n > 0.
 */
template <typename T>
T ModAdd(const T& a, const T& b, const T& n) {
    return a >= n - b ? a - n + b : a + b;
}

/**
 * Compute (a - b) % n.
 *
 * Require 0 <= a < n, 0 <= b < n and n > 0.
 */
template <typename T>
T ModSub(const T& a, const T& b, const T& n) {
    return a < b ? a - b + n : a - b;
}

/**
 * Compute (a * b) % n without overflow.
 *
 * Require n > 0.
 */
template <typename T>
T ModMul(T a, T b, const T& n) {
    T res = 0;

    while (a != 0) {
        if (a & 1) {
            if (b >= n - res) {
                res += b;
                res -= n;
            } else {
                res += b;
            }
        }
        a >>= 1;

        // Double b, modulo n.
        if (b >= n - b) {
            b += b;
            b -= n;
        } else {
            b += b;
        }

        if (b == 0) break;
    }

    return res;
}
inline int32_t ModMul(int32_t a, int32_t b, int32_t n) {
    return static_cast<int64_t>(a) * b % n;
}
inline uint32_t ModMul(uint32_t a, uint32_t b, uint32_t n) {
    return static_cast<uint64_t>(a) * b % n;
}
inline BigInt ModMul(const BigInt& a, const BigInt& b, const BigInt& n) {
    return a * b % n;
}

/**
 * Compute the modular multiplicative inverse.
 *
 * A modular multiplicative inverse of an integer a is an integer x such that
 * the product ax is congruent to 1 with respect to the modulus n, i.e.,
 *
 *   ax = 1 (mod n)
 *
 * If d is the greatest common divisor of a and n then the linear congruence
 * ax = b (mod n) has solutions if and only if d divides b. If d divides b, then
 * there are exactly d solutions.
 *
 * In this case, it means a and n must be coprime, otherwise the equation has no
 * solution.
 *
 * Return false if a and n is not coprime.
 */
template <typename T>
bool ModInverse(const T& a, const T& n, T* x) {
    CHECK(n > 0);
    CHECK(x);

    // Compute
    //      a * d + b * n = GCD(a, n) = 1
    // by ExtendedGCD.
    // Then we have
    //      a * d = 1 - b * n,
    // and
    //      a * d = 1 (mod n)
    T y;
    T d = ExtendedGCD(Mod(a, n), n, x, &y);
    if (d != 1) return false;

    *x = Mod(*x, n);
    return true;
}

/**
 * Compute the modular power: (a ^ b) % n.
 * Require n > 0.
 */
template <typename T>
T ModPow(T a, T b, const T& n) {
    CHECK(n > 0);

    T r = 1;

    while (b > 0) {
        if (b % 2 == 1) {
            r = ModMul(a, r, n);
        }
        a = ModMul(a, a, n);
        b >>= 1;
    }

    return (n == 1) ? 0 : r;
}

} // namespace cl

#endif // CODELIBRARY_MATH_MODULAR_MODULAR_H_
