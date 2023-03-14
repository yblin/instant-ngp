//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_FRACTION_FAREY_SEQUENCE_TEST_H_
#define CODELIBRARY_TEST_MATH_FRACTION_FAREY_SEQUENCE_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/math/fraction/farey_sequence.h"

namespace cl {
namespace test {

TEST(FractionUtilTest, FareySequeceTest) {
    Array<Fraction<int>> fractions;
    FareySequence(5, &fractions);

    const Array<Fraction<int>> res = {
        Fraction<int>(0, 1),
        Fraction<int>(1, 5),
        Fraction<int>(1, 4),
        Fraction<int>(1, 3),
        Fraction<int>(2, 5),
        Fraction<int>(1, 2),
        Fraction<int>(3, 5),
        Fraction<int>(2, 3),
        Fraction<int>(3, 4),
        Fraction<int>(4, 5),
        Fraction<int>(1, 1)
    };

    ASSERT_EQ_RANGE(fractions.begin(), fractions.end(), res.begin(), res.end());
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_FRACTION_FAREY_SEQUENCE_TEST_H_
