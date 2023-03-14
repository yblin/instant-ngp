//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_ISQRT_H_
#define CODELIBRARY_MATH_ISQRT_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "codelibrary/math/number/bigint.h"

namespace cl {

/**
 * Get the greatest integer which is less than or equal to the square root of
 * the given positive integer.
 */
inline uint32_t ISqrt(uint32_t n) {
    uint32_t is = static_cast<uint32_t>(std::sqrt(static_cast<double>(n)));
    is -= static_cast<uint32_t>(is * is > n);
    return is;
}

inline int32_t ISqrt(int32_t n) {
    CHECK(n >= 0);

    return static_cast<int32_t>(ISqrt(static_cast<uint32_t>(n)));
}

inline uint64_t ISqrt(uint64_t n) {
    uint64_t y = static_cast<uint64_t>(std::sqrt(static_cast<double>(n)));
    y -= (y * y > n);
    if (2 * y < n - y * y) ++y;
    if (y == 4294967296ULL) --y;
    return y;
}

inline int64_t ISqrt(int64_t n) {
    CHECK(n >= 0);

    return static_cast<int64_t>(ISqrt(static_cast<uint64_t>(n)));
}

inline BigInt ISqrt(const BigInt& n) {
    CHECK(n >= 0);

    return n.Sqrt();
}

} // namespace cl

#endif // CODELIBRARY_MATH_ISQRT_H_
