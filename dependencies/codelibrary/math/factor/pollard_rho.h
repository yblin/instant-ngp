//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_FACTOR_POLLARD_RHO_H_
#define CODELIBRARY_MATH_FACTOR_POLLARD_RHO_H_

#include "codelibrary/math/common_factor.h"
#include "codelibrary/math/isqrt.h"
#include "codelibrary/math/prime/is_prime.h"

namespace cl {
namespace factor {

/**
 * Get a factor of 'n' via Pollard's rho factorization algorithm.
 *
 * The expected time complex is O(n ^ 0.25).
 *
 * Note that Pollard's rho may fail. Subsequent attempts can be made by changing
 * the 'x0' and 'c'.
 */
template <typename T>
T PollardRho(const T& n, const T& x0, const T& c) {
    CHECK(n >= 2);

    T x = x0, q = 1, g = 1, xs, y;
    int m = 128;
    int iter = 1;
    while (g == 1) {
        y = x;
        for (int i = 1; i < iter; ++i) {
            x = ModAdd(ModMul(x, x, n), c, n);
        }
        int k = 0;
        while (k < iter && g == 1) {
            xs = x;
            for (int i = 0; i < m && i < iter - k; i++) {
                x = ModAdd(ModMul(x, x, n), c, n);
                if (y >= x)
                    q = ModMul(q, y - x, n);
                else
                    q = ModMul(q, x - y, n);
            }
            if (q == 0)
                g = n;
            else
                g = GCD(q, n);
            k += m;
        }
        iter *= 2;
    }

    if (g == n) {
        do {
            xs = ModAdd(ModMul(xs, xs, n), c, n);
            if (xs >= y) {
                g = GCD(xs - y, n);
            } else {
                g = GCD(y - xs, n);
            }
        } while (g == 1);
    }

    return g == n ? 1 : g;
}

/**
 * Get a factor of 'n' via Pollard's rho factorization algorithm.
 *
 * It will call Pollard's rho at most max_iters times.
 *
 * If no factor is found the function will return 1.
 */
template <typename T>
T PollardRho(const T& n, int max_iters = 40000) {
    CHECK(n >= 2);
    CHECK(max_iters > 0);

    if (IsPrime(n)) return n;

    std::mt19937 random;
    for (int i = 0; i < max_iters; ++i) {
        T factor = PollardRho(n, T(random()), T(random()));
        if (factor != 1) return factor;
    }

    return 1;
}

} // namespace factor
} // namespace cl

#endif // CODELIBRARY_MATH_FACTOR_POLLARD_RHO_H_
