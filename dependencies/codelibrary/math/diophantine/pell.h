//
// Copyright 2012-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_DIOPHANTINE_PELL_H_
#define CODELIBRARY_MATH_DIOPHANTINE_PELL_H_

#include "codelibrary/math/number/bigint.h"

namespace cl {
namespace diophantine {

/**
 * Solve Pell's equation: x^2 - n*y^2 = 1,
 * where n is a given non-square integer, x and y are positive integer.
 *
 * The function uses a continued fraction method to solve the Pell equation.
 * Note that, the number of digits in the solution may be as large as sqrt(n).
 *
 * Return false if n is the square integer.
 */
bool Pell(const BigInt& n, BigInt* x, BigInt* y) {
    CHECK(n > 0 && x && y);

    BigInt sqrt_n = n.Sqrt();

    if (sqrt_n * sqrt_n == n) return false;

    // If the period length is even, x / y equal to the (period - 1)-th
    // convergent to continued fraction of sqrt(n);
    // If the period length is odd, x / y equal to the (2 * period - 1)-th
    // convergent to continued fraction of sqrt(n).
    BigInt m = 0;
    BigInt d = 1;
    BigInt a = sqrt_n;
    BigInt p0 = 1, p1 = sqrt_n, q0 = 0, q1 = 1, p2, q2;

    for (int i = 0; ; ++i) {
        m = d * a - m;
        d = (n - m * m) / d;
        a = (m + sqrt_n) / d;

        p2 = a * p1 + p0;
        q2 = a * q1 + q0;

        p0 = p1;
        p1 = p2;
        q0 = q1;
        q1 = q2;

        if (p1 * p1 - q1 * q1 * n == 1) {
            *x = p1 >= 0 ? p1 : -p1;
            *y = q1 >= 0 ? q1 : -q1;
            return true;
        }
    }

    return false;
}

} // namespace diophantine
} // namespace cl

#endif // CODELIBRARY_MATH_DIOPHANTINE_PELL_H_
