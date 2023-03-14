//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_PRIME_EULER_TOTIENT_TABLE_H_
#define CODELIBRARY_MATH_PRIME_EULER_TOTIENT_TABLE_H_

#include "codelibrary/base/array.h"

namespace cl {

/**
 * Generate Euler totient table between 1 to n in linear time.
 *
 * Euler totient, Phi(n), is the positive integers up to a given integer n that
 * are relatively prime to n.
 */
void EulerTotientTable(int n, Array<int>* phi) {
    CHECK(n > 0);
    CHECK(phi);

    Array<int> primes;
    phi->resize(n, 0);
    for (int i = 2; i < n; ++i) {
        if ((*phi)[i] == 0) {
            (*phi)[i] = i - 1;
            primes.push_back(i);
        }

        for (int p : primes) {
            if (i > (n - 1) / p) break;
            if (i % p == 0) {
                (*phi)[i * p] = (*phi)[i] * p;
                break;
            }

            (*phi)[i * p] = (*phi)[i] * (p - 1);
        }
    }
    (*phi)[1] = 1;
}

} // namespace cl

#endif // CODELIBRARY_MATH_PRIME_EULER_TOTIENT_TABLE_H_
