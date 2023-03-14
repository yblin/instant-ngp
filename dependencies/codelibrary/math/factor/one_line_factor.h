//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_FACTOR_ONE_LINE_FACTOR_H_
#define CODELIBRARY_MATH_FACTOR_ONE_LINE_FACTOR_H_

#include "codelibrary/math/common_factor.h"
#include "codelibrary/math/factor/trial_division.h"
#include "codelibrary/math/isqrt.h"

namespace cl {
namespace factor {

/**
 * Hart's one line factoring algorithm [1] is very efficient for n = pq
 * (bits(n) <= 42) and bits of p and q are close.
 *
 * Here, we adopt the modify version proposed in [2], which is 37.5% faster than
 * the origianl one.
 *
 * Reference:
 * [1] Hart W. A one line factoring algorithm[J]. Journal of The Australian
 *     Mathematical Society, 2012, 92(01): 61-69.
 * [2] Tejas Gopalakrishna and Yichi Zhang. Analysis of the One Line Factoring
 *     Algorithm, 2019.
 */
inline uint64_t OneLineFactor(uint64_t n) {
    CHECK(n >= 2);
    CHECK(n <= (1ULL << 42));

    // N should be odd.
    if (n % 2 == 0) return 2;

    uint64_t s, m, kn, sqrt_m, t;

    for (uint64_t k = 1; k <= n; ++k) {
        t = k % 8;
        if (t != 2 && t != 4 && t != 6) {
            kn = k * n;
            s = ISqrt(kn);
            if (s * s < kn) ++s;
            m = s * s - kn;

            sqrt_m = ISqrt(m);
            if (sqrt_m * sqrt_m == m) {
                uint64_t res = GCD(s - sqrt_m, n);
                return res == 1 ? n : res;
            }
        }
    }

    // It always return a correct factor for odd n below 10^15.
    CHECK(false) << "Code should not reach here.";
    return 1;
}

} // namespace factor
} // namespace cl

#endif // CODELIBRARY_MATH_FACTOR_ONE_LINE_FACTOR_H_
