//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_FRACTION_FAREY_SEQUENCE_H_
#define CODELIBRARY_MATH_FRACTION_FAREY_SEQUENCE_H_

#include "codelibrary/base/array.h"
#include "codelibrary/math/fraction/fraction.h"

namespace cl {

/**
 * Generate the first 'n' Farey sequence.
 *
 * The Farey sequence is the sequence of completely reduced fractions between
 * 0 and 1 which, when in lowest terms, have denominators less than or equal to
 * n, arranged in order of increasing size.
 *
 * The Farey sequences of orders 1 to 5 are:
 *   F1 = {0/1, 1/1}
 *   F2 = {0/1, 1/2, 1/1}
 *   F3 = {0/1, 1/3, 1/2, 2/3, 1/1}
 *   F4 = {0/1, 1/4, 1/3, 1/2, 2/3, 3/4, 1/1}
 *   F5 = {0/1, 1/5, 1/4, 1/3, 2/5, 1/2, 3/5, 2/3, 3/4, 4/5, 1/1}
 */
inline void FareySequence(int n, Array<Fraction<int>>* farey_sequence) {
    CHECK(farey_sequence);
    CHECK(n > 0);

    farey_sequence->clear();
    farey_sequence->reserve(n);

    farey_sequence->emplace_back(0, 1);
    unsigned int unsigned_n = static_cast<unsigned int>(n);
    unsigned int a = 0, b = 1, c = 1, d = unsigned_n;
    while (c <= unsigned_n) {
        unsigned int k = (unsigned_n + b) / d;
        unsigned int t1 = c;
        unsigned int t2 = d;
        c = k * c - a;
        d = k * d - b;
        a = t1;
        b = t2;
        farey_sequence->emplace_back(a, b);
    }
}

} // namespace cl

#endif // CODELIBRARY_MATH_FRACTION_FAREY_SEQUENCE_H_
