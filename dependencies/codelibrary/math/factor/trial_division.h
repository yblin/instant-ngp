//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_FACTOR_TRIAL_DIVISION_H_
#define CODELIBRARY_MATH_FACTOR_TRIAL_DIVISION_H_

#include <cstdint>

#include "codelibrary/math/isqrt.h"
#include "codelibrary/math/prime/wheel_sieve.h"

namespace cl {
namespace factor {

/**
 * Retrun a small factor of n. If n is prime, return itself.
 *
 * It returns the correct result only when n <= 211 * 211.
 */
template <typename T>
T SmallFactor(const T& n) {
    CHECK(n > 0);

    if (n == 1) return 1;
    if (n % 2 == 0) return 2;

    // The following code is optimized for BigInt.
    static const uint32_t small_factors1[8] = { 3, 5, 7, 11, 13, 17, 19, 23 };
    static const uint32_t pp1 = 3 * 5 * 7 * 11 * 13 * 17 * 19 * 23;
    uint32_t m = n % pp1;
    for (uint32_t i : small_factors1) {
        if (m % i == 0) return i;
    }
    if (n <= 23 * 23) return n;

    static const uint32_t small_factors2[5] = { 29, 31, 37, 41, 43 };
    static const uint32_t pp2 = 29 * 31 * 37 * 41 * 43;
    m = n % pp2;
    for (uint32_t i : small_factors2) {
        if (m % i == 0) return i;
    }
    if (n <= 43 * 43) return n;

    // Check the prime factor in [47, 67].
    static const uint32_t small_factors3[5] = { 47, 53, 59, 61, 67 };
    static const uint32_t pp3 = 47 * 53 * 59 * 61 * 67;
    m = n % pp3;
    for (uint32_t i : small_factors3) {
        if (m % i == 0) return i;
    }
    if (n <= 67 * 67) return n;

    // Check the prime factor in [71, 211].
    static const uint32_t small_factors4[7][4] = {
        { 71,  73,  79,  83  },
        { 89,  97,  101, 103 },
        { 107, 109, 113, 127 },
        { 131, 137, 139, 149 },
        { 151, 157, 163, 167 },
        { 173, 179, 181, 191 },
        { 193, 197, 199, 211 }
    };
    static const uint32_t pp4[7] = {
        71  * 73  * 79  * 83,
        89  * 97  * 101 * 103,
        107 * 109 * 113 * 127,
        131 * 137 * 139 * 149,
        151 * 157 * 163 * 167,
        173 * 179 * 181 * 191,
        193 * 197 * 199 * 211
    };
    for (int k = 0; k < 7; ++k) {
        uint32_t m = n % pp4[k];

        for (int i = 0; i < 4; ++i) {
            if (m % small_factors4[k][i] == 0) return small_factors4[k][i];
        }
        if (n <= small_factors4[k][3] * small_factors4[k][3]) return n;
    }

    return n;
}

/**
 * Trial division to find a factor (<= bound) of n.
 */
template <typename T>
T TrialDivision(const T& n, int bound) {
    CHECK(n >= 2);
    CHECK(bound > 0);

    T small_factor = SmallFactor(n);
    if (small_factor < n) return small_factor;

    const int tested = 211 * 211;
    if (n <= tested || bound <= tested) return n;

    T sqrt_n = ISqrt(n);
    if (sqrt_n * sqrt_n == n) return sqrt_n;

    prime::WheelSieve30 sieve(bound);
    auto i = sieve.Find(211);
    for (; i != sieve.end(); ++i) {
        int p = *i;
        if (T(p) > sqrt_n) break;
        if (sieve.IsPrime(p) && n % p == 0) return p;
    }

    return 1;
}

/**
 * Trial division to find a factor of n.
 */
template <typename T>
T TrialDivision(const T& n) {
    CHECK(n >= 2);

    return TrialDivision(n, ISqrt(n));
}

} // namespace factor
} // namespace cl

#endif // CODELIBRARY_MATH_FACTOR_TRIAL_DIVISION_H_
