//
// Copyright 2019 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_NUMBER_FIBONACCI_H_
#define CODELIBRARY_MATH_NUMBER_FIBONACCI_H_

#include <algorithm>
#include <string>

#include "codelibrary/math/number/bigint.h"

namespace cl {

/**
 * Compute the n-th fibonacci number.
 *
 * It adopts a matrix multiplication method to compute the n-th fibonacci in
 * log(n) time. It also uses Cassini's formula to optimize the speed.
 */
BigInt Fibonacci(int n) {
    assert(n >= 0);

    if (n == 0) return 1;

    // Convert n to binary string.
    std::string binary_n;
    while (n) {
        if (n % 2 == 0)
            binary_n += '0';
        else
            binary_n += '1';
        n /= 2;
    }
    std::reverse(binary_n.begin(), binary_n.end());

    BigInt x = 1, y = 0, xx, yy, xy;
    char prev = '0';
    for (char cur : binary_n) {
        xx = x * x;
        yy = y * y;
        xy = xx - yy;
        if (prev != '0')
            xy += 1;
        else
            xy -= 1;

        if (cur != '0') {
            x = xx + xy + xy;
            y = xx + yy;
        } else {
            x = xx + yy;
            y = xy + xy - yy;
        }
        prev = cur;
    }

    return y;
}

} // namespace cl

#endif // CODELIBRARY_MATH_NUMBER_FIBONACCI_H_
