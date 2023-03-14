//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_PRIME_MILLER_RABIN_TEST_H_
#define CODELIBRARY_MATH_PRIME_MILLER_RABIN_TEST_H_

#include "codelibrary/base/array.h"
#include "codelibrary/math/modular/modular.h"

namespace cl {
namespace prime {

/**
 * Test if a given number is a 'probable' prime number by Miller-Rabin
 * algorithm.
 *
 * The running time is O(k(logn)^3) (k: the number of witnesses).
 * It had been proven that:
 *
 * if n < 2^32, it is enough to test {2, 7, 61}. （found by Gerhard Jaeschke)
 * if n < 2^64, it is enough to test {2, 325, 9375, 28178, 450775, 9780504,
 * 1795265022}. (found by Jim Sinclair)
 *
 * Otherwise, user can generate K random witnesses between 2 to n - 2. In this
 * case, Miller–Rabin primality test declares n be a probably prime with a
 * probability at least (1 - 4^−K).
 */
template <typename T>
bool MillerRabinTest(const T& n, const Array<T>& witnesses) {
    CHECK(n >= 0);

    // Miller-rabin only work for n > 1 and n % 2 == 1.
    if (n <= 1) return false;
    if (n % 2 == 0) return false;

    // Write n - 1 as 2^s * d by factoring powers of 2 from n-1.
    int s = 0;
    T d = n - 1;
    for (; d % 2 == 0; ++s, d >>= 1); // loop.

    for (T a : witnesses) {
        CHECK(a >= 2) << "Witness shoud not be less than 2.";
        a = a % n;
        if (a == 0) return true;
        a = ModPow(a, d, n);

        if (a == 1 || a == n - 1) continue;

        int r = 0;
        for (r = 0; r < s - 1; ++r) {
            a = ModMul(a, a, n);
            if (a == n - 1) break;
        }
        if (r == s - 1) return false;
    }
    return true;
}

} // namespace prime
} // namespace cl

#endif // CODELIBRARY_MATH_PRIME_MILLER_RABIN_TEST_H_
