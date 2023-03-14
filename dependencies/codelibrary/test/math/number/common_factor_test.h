//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_COMMON_FACTOR_TEST_H_
#define CODELIBRARY_TEST_MATH_COMMON_FACTOR_TEST_H_

#include <cstdint>
#include <limits>

#include "codelibrary/base/testing.h"
#include "codelibrary/math/common_factor.h"

namespace cl {
namespace test {

TEST(CommonFactor, ExtendedGCDTest) {
    int x, y;
    int gcd1 = ExtendedGCD(105, 196, &x, &y);

    ASSERT_EQ(x * 105 + y * 196, gcd1);

    int n = std::numeric_limits<int>::max();
    int gcd2 = ExtendedGCD(n, n, &x, &y);
    ASSERT_EQ(n * x + n * y, gcd2);

    int gcd3 = ExtendedGCD(n, n - 3, &x, &y);
    ASSERT_EQ(n * x + (n - 3) * y, gcd3);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_COMMON_FACTOR_TEST_H_
