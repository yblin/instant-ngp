//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_FACTOR_FACTOR_H_
#define CODELIBRARY_MATH_FACTOR_FACTOR_H_

#include "codelibrary/math/factor/one_line_factor.h"
#include "codelibrary/math/factor/pollard_rho.h"
#include "codelibrary/math/factor/squfof.h"
#include "codelibrary/math/factor/trial_division.h"

namespace cl {

/**
 * Return a factor of n. If the return number equals to n, n is a prime number.
 *
 * It returns 1 if factorization fails.
 */
template <typename T>
T Factor(const T& n) {
    CHECK(n >= 2);

    T small_factor = factor::SmallFactor(n);
    if (small_factor != n) return small_factor;

    if (n <= (1ULL << 42)) return factor::OneLineFactor(n);
    if (n < (1ULL << 62)) return factor::SQUFOF(n);

    return factor::PollardRho(n);
}

} // namespace cl

#endif // CODELIBRARY_MATH_NUMBER_FACTOR_FACTOR_H_
