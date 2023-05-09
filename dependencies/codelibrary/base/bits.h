//
// Copyright 2015-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// This file defines some common bit utilities.
//

#ifndef CODELIBRARY_BASE_BITS_H_
#define CODELIBRARY_BASE_BITS_H_

#include <limits>
#include <type_traits>

#include "codelibrary/base/log.h"

namespace cl {
namespace bits {

/**
 * Return the integer i such as 2^i <= n < 2^(i+1).
 */
template <typename IntType>
int Log2Floor(IntType n) {
    static_assert(std::is_integral<IntType>::value, "n should be an integer.");
    CHECK(n > 0);

    int start = 0;
    switch (std::numeric_limits<IntType>::digits) {
    case 7:
    case 8:
        start = 2;
        break;
    case 15:
    case 16:
        start = 3;
        break;
    case 31:
    case 32:
        start = 4;
        break;
    case 63:
    case 64:
        start = 5;
        break;
    default:
        CHECK(false) << "Over-sized type.";
    }

    int log = 0;
    for (int i = start; i >= 0; --i) {
        int shift = (1 << i);
        IntType x = n >> shift;
        if (x != 0) {
            n = x;
            log += shift;
        }
    }

    return log;
}

/**
 * Return the integer i such as 2^(i-1) < n <= 2^i.
 */
template <typename IntType>
IntType Log2Ceil(IntType n) {
    CHECK(n >= 0);
    if (n == 0 || n == 1) return 0;

    return IntType(1) + Log2Floor(n - 1);
}

/**
 * Return the integer 2^i such as 2^i <= n < 2^(i+1).
 */
template <typename IntType>
IntType Power2Floor(IntType n) {
     return IntType(1) << Log2Floor(n);
}

/**
 * Return the integer 2^i such as 2^(i-1) < n <= 2^i.
 */
template <typename IntType>
IntType Power2Ceil(IntType n) {
    int n_bits = Log2Ceil(n);
    CHECK(n_bits < std::numeric_limits<IntType>::digits);

    return IntType(1) << n_bits;
}

/**
 * Return the number of bits in integer n (excluding leading zeros)
 *
 * Equal to n_bits - __builtin_clz(n)
 */
template <typename IntType>
int CountBits(IntType n) {
    static_assert(std::is_integral<IntType>::value, "n should be an integer.");

    if (n == 0) return 0;
    if (n < 0) return std::numeric_limits<IntType>::digits;
    return Log2Floor(n) + 1;
}

/**
 * Count the number of bits '1' of 'n'
 */
inline int CountOnes(uint8_t n) {
    static const int table[16] = {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4
    };

    return table[n >> 4] + table[n & 0x0F];
}

} // namespace bits
} // namespace cl

#endif // CODELIBRARY_BASE_BITS_H_
