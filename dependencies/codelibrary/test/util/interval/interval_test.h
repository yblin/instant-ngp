//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_INTERVAL_INTERVAL_TEST_H_
#define CODELIBRARY_TEST_UTIL_INTERVAL_INTERVAL_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/util/interval/interval.h"

namespace cl {
namespace test {

TEST(IntervalTest, OverlapTest) {
    Interval<int> a1(2, 3, Interval<int>::OPEN, Interval<int>::CLOSED);
    Interval<int> b1(1, 2, Interval<int>::CLOSED, Interval<int>::OPEN);
    ASSERT_FALSE(a1.Overlap(b1));

    Interval<int> a2(2, 3, Interval<int>::CLOSED, Interval<int>::OPEN);
    Interval<int> b2(2, 5, Interval<int>::CLOSED, Interval<int>::OPEN);
    ASSERT(a2.Overlap(b2));

    Interval<int> a3(2, 3, Interval<int>::OPEN, Interval<int>::CLOSED);
    Interval<int> b3(3, 5, Interval<int>::CLOSED, Interval<int>::OPEN);
    ASSERT(a3.Overlap(b3));

    // [1 3) + [3 5) -> false.
    Interval<int> a4(1, 3, Interval<int>::CLOSED, Interval<int>::OPEN);
    Interval<int> b4(3, 5, Interval<int>::CLOSED, Interval<int>::OPEN);
    ASSERT_FALSE(a4.Overlap(b4));
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_INTERVAL_INTERVAL_TEST_H_
