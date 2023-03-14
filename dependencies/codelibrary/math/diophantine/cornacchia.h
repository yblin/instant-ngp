//
// Copyright 2012-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_DIOPHANTINE_CORNACCHIA_H_
#define CODELIBRARY_MATH_DIOPHANTINE_CORNACCHIA_H_

#include <numeric>

#include "codelibrary/base/log.h"
#include "codelibrary/math/common_factor.h"
#include "codelibrary/math/isqrt.h"
#include "codelibrary/math/modular/modular.h"

namespace cl {
namespace diophantine {

/**
 * Cornacchia's algorithm to find primitive solution to x^2 + d*y^2 = n,
 * where 0 < d < n, d and n are relatively positive prime integers.
 *
 * e.g. Cornacchia(4, 1733, &x, &y) gives x = 17, y = 19; that is,
 *   17^2 + 4 * 19^2 = 1733.
 *
 * Return false if no solution.
 */
template <typename T>
bool Cornacchia(const T& d, const T& n, T* x, T* y) {
    CHECK(x && y);
    CHECK(0 < d && d < n);
    CHECK(GCD(d, n) == 1) << "d and n should be coprime.";

    for (T t = 1; t < n; ++t) {
        // Find all positive solution (less than n) to t^2 + d = 0 (mod n).
        // Note that, if n is odd prime, it can be solved by modular square root
        // algorithms.
        if (ModAdd(ModMul(t, t, n), d, n) != 0) continue;

        // For each solution t, use Euclidean algorithm to find the first
        // remainder b less than sqrt(n).
        T tmp, a = n, b = t;

        // tb >= n / b, i.e., b * b <= n without overflow.
        while (b != 0 && b >= n / b) {
            tmp = b;
            b = a % b;
            a = tmp;
        }
        CHECK(b * b <= n);

        // If s = sqrt((n - b^2)/d) is an integer, (b, s) is a solution.
        if ((n - b * b) % d == 0) {
            T s = ISqrt((n - b * b) / d);
            if (s * s * d == n - b * b) {
                *x = b;
                *y = s;
                return true;
            }
        }
    }
    return false;
}

} // namespace diophantine
} // namespace cl

#endif // CODELIBRARY_MATH_DIOPHANTINE_CORNACCHIA_H_
